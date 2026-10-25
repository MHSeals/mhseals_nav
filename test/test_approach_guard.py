"""approach_guard: a hull closing in holds, static buoys and docks passed close never do."""
import json
from pathlib import Path
import time

from mhseals_nav.approach_guard import Guard, hull_gap, lethal_points
import numpy
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/approach_guard.yaml')
                     .read_text())['/**']['ros__parameters']
RES = 0.15
DT = 1.0 / CFG['rate_hz']


def cells(*boxes):
    """World centres of the 0.15 m cells covering axis-aligned boxes (cx, cy, half_x, half_y)."""
    out = []
    for cx, cy, hx, hy in boxes:
        xs = numpy.arange(numpy.floor((cx - hx) / RES), numpy.ceil((cx + hx) / RES))
        ys = numpy.arange(numpy.floor((cy - hy) / RES), numpy.ceil((cy + hy) / RES))
        gx, gy = numpy.meshgrid(xs, ys)
        out.append(numpy.column_stack(((gx.ravel() + 0.5) * RES, (gy.ravel() + 0.5) * RES)))
    return numpy.concatenate(out) if out else numpy.zeros((0, 2))


def run(world, seconds, speed=0.4, yaw_rate=0.0, autonomy=True, flicker=0.0, seed=0):
    """Drive along +x; ``world(t)`` gives lethal boxes. Return when hold requests went out."""
    rng = numpy.random.default_rng(seed)
    guard, sent = Guard(CFG), []
    x = y = yaw = 0.0
    for k in range(int(seconds / DT)):
        t = k * DT
        points = cells(*world(t))
        if flicker and len(points):
            points = points[rng.random(len(points)) > flicker]
        if autonomy:
            guard.command(t)
        guard.observe(t, points, (x, y, yaw))
        if guard.tick(t):
            sent.append(round(t, 1))
        x += speed * DT * numpy.cos(yaw)
        y += speed * DT * numpy.sin(yaw)
        yaw += yaw_rate * DT
    return sent


def gate(t):
    return [(5.0, 0.9, 0.2, 0.2), (5.0, -0.9, 0.2, 0.2), (12.0, 0.9, 0.2, 0.2)]


def overtaker(t):
    """S-overtake: 1.0 x 0.5 m, from 8 m behind and 1.5 m off, at 1.0 m/s (0.05 m/s in)."""
    return [(-8.0 + 1.0 * t, -1.5 + 0.05 * t, 0.5, 0.25)]


def test_gate_buoys_passed_close_never_hold():
    # 0.45 m off the hull side, past and behind on both quarters, with cells flickering.
    assert run(gate, 40.0, flicker=0.3) == []
    assert run(gate, 40.0, yaw_rate=0.05, flicker=0.3, seed=1) == []


def test_a_buoy_the_lidar_loses_and_finds_again_never_holds():
    def blinking(t):
        # Beside the hull, out of the lidar's cone from 8 s to 20 s (sim: 7-19 s).
        return [] if 8.0 <= t < 20.0 else [(0.0, 1.3, 0.2, 0.2)]
    assert run(blinking, 30.0, speed=0.0) == []


def test_obstacles_seen_at_start_are_learnt_first():
    assert run(gate, 10.0, speed=0.0) == []
    assert run(lambda t: [(-2.0, 1.4, 0.2, 0.2)], 10.0, speed=0.0) == []


def test_a_dock_alongside_never_holds():
    def dock(t):
        return [(3.0, 1.2, 4.0, 0.3), (8.0, 0.0, 0.3, 2.0)]
    assert run(dock, 15.0, speed=0.3, flicker=0.2) == []


def test_overtaker_holds_well_before_contact():
    sent = run(overtaker, 25.0)
    assert sent
    t = sent[0]
    gap = hull_gap(cells(*overtaker(t)), (0.4 * t, 0.0, 0.0), (0.5315, 0.4475)).min()
    assert gap > 1.5
    # Never released: it repeats every repeat_s while the approach lasts.
    assert all(b - a >= CFG['repeat_s'] - 1e-6 for a, b in zip(sent, sent[1:]))


def test_head_on_boat_holds_too():
    def head_on(t):
        return [(8.0 - 0.8 * t, 0.3, 0.5, 0.25)]
    assert run(head_on, 10.0)


def test_no_hold_without_autonomy():
    assert run(overtaker, 25.0, autonomy=False) == []


def test_blind_spell_is_not_motion():
    guard = Guard(CFG)
    guard.command(0.0)
    for k in range(10):
        guard.observe(k * DT, cells(*gate(0)), (0.0, 0.0, 0.0))
    guard.blind()
    for k in range(10, 20):
        guard.command(k * DT)
        guard.observe(k * DT, cells(*gate(0)), (0.0, 0.0, 0.0))
        assert not guard.tick(k * DT)


def test_lethal_points_reads_world_cells():
    grid = numpy.zeros((40, 40), numpy.int8)
    grid[10, 20] = 100
    grid[30, 30] = 99
    pts = lethal_points(grid, 0.5, (-10.0, -10.0), (0.0, -5.0), 2.0)
    assert pts.tolist() == [[0.25, -4.75]]


def test_node_loads_its_config_and_holds_with_a_token():
    rclpy = pytest.importorskip('rclpy')
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.approach_guard import node_class
    rclpy.init()
    try:
        node = node_class()()
        assert {k: node.cfg[k] for k in CFG} == CFG
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        probe = rclpy.create_node('approach_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(probe)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not got:
            node.logic.approach_at = time.monotonic()   # no costmap: the decision alone
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        assert got and got[0]['op'] == 'hold' and got[0]['cause'] == 'approach'
        assert got[0]['token'] == node.token
        node.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.try_shutdown()
