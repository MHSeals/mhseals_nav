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
ENFORCE = {**CFG, 'enforce': True}   # as navigation.launch.py runs it in sim
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


def run(world, seconds, speed=0.4, yaw_rate=0.0, autonomy=True, flicker=0.0, seed=0,
        cfg=ENFORCE, seen=None, drop=0.0):
    """Drive along +x; ``world(t)`` gives lethal boxes. Return when hold requests went out.

    ``seen`` collects the times an approach was reported (reason set), held or not;
    ``drop`` is the share of snapshots that never arrive.
    """
    rng = numpy.random.default_rng(seed)
    guard, sent = Guard(cfg), []
    x = y = yaw = 0.0
    for k in range(int(seconds / DT)):
        t = k * DT
        points = cells(*world(t))
        if flicker and len(points):
            points = points[rng.random(len(points)) > flicker]
        if autonomy:
            guard.command(t)
        if not (drop and rng.random() < drop):
            guard.observe(t, points, (x, y, yaw))
        if seen is not None and guard.reason(t):
            seen.append(round(t, 1))
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


def test_log_only_reports_the_approach_and_never_holds():
    assert CFG['enforce'] is False   # the boat: log-only until water bags replay clean
    seen = []
    assert run(overtaker, 25.0, cfg=CFG, seen=seen) == []
    assert seen and seen[0] == run(overtaker, 25.0)[0]


def test_scattered_clutter_never_chains_into_a_hold():
    def clutter(spots):
        def world(t):
            # Open water for warmup, then three fresh specks a snapshot (spray, wake)
            # anywhere within 4.5 m of the boat, each new and none of them moving on.
            if t < 5.0:
                return []
            k = int(round(t * CFG['rate_hz']))
            return [(0.4 * t + x, y, 0.05, 0.05) for x, y in spots[3 * k:3 * k + 3]
                    if abs(y) > 0.6 or abs(x) > 0.7]
        return world
    for seed in range(5):
        world = clutter(numpy.random.default_rng(seed).uniform(-4.5, 4.5, (200, 2)))
        assert run(world, 9.0) == []
        # Without the chain and cluster rules the same clutter would hold.
        assert run(world, 9.0, cfg={**ENFORCE, 'max_speed': 1e9, 'min_cluster': 1})


def specks(rng, count):
    """``count`` fresh specks a snapshot (spray, wake) within 4.5 m of the boat, after warmup."""
    spots = rng.uniform(-4.5, 4.5, (20000, 2))

    def world(t):
        if t < 5.0:
            return []
        k = int(round(t * CFG['rate_hz']))
        return [(0.4 * t + x, y, 0.05, 0.05) for x, y in spots[count * k:count * (k + 1)]
                if abs(y) > 0.6 or abs(x) > 0.7]
    return world


@pytest.mark.parametrize('count', [1, 2, 3])
def test_overtaker_holds_through_clutter(count):
    alone = run(overtaker, 25.0)[0]
    for seed in range(10):
        clutter = specks(numpy.random.default_rng(seed), count)
        sent = run(lambda t: overtaker(t) + clutter(t), 25.0)
        assert sent and sent[0] <= alone + 2.5, (seed, sent[:1], alone)


@pytest.mark.parametrize('count', [10, 20, 40])
def test_dense_clutter_alone_never_holds(count):
    for seed in range(10):
        assert run(specks(numpy.random.default_rng(seed), count), 26.0) == [], seed


@pytest.mark.parametrize('drop,latest', [(0.2, 8.0), (0.3, 8.5), (0.4, 9.0)])
def test_overtaker_holds_through_snapshot_dropouts(drop, latest):
    for seed in range(10):
        sent = run(overtaker, 25.0, drop=drop, seed=seed)
        assert sent and sent[0] <= latest, (seed, sent[:1])


def test_a_chain_survives_miss_s_without_a_new_cell_and_no_longer():
    guard = Guard(ENFORCE)
    guard.chains = [[8.0, 9.0, 0.0, 0.0, 1.0, 5, 5]]
    none = numpy.zeros((0, 2)), numpy.zeros(0)
    guard.follow(9.0 + CFG['miss_s'], *none)
    assert guard.chains
    guard.follow(9.0 + CFG['miss_s'] + 0.5 / CFG['rate_hz'], *none)
    assert guard.chains == []


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
    assert guard.chains == []
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


def spin_node(enforce, seconds=2.5):
    """Run the node with an approach forced; return (hold requests, reports) it published."""
    rclpy = pytest.importorskip('rclpy')
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.approach_guard import node_class
    rclpy.init()
    try:
        node = node_class()(parameter_overrides=[
            rclpy.parameter.Parameter('enforce', value=enforce)])
        assert {k: node.cfg[k] for k in CFG if k != 'enforce'} == \
            {k: v for k, v in CFG.items() if k != 'enforce'}
        assert node.cfg['enforce'] is enforce
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        probe = rclpy.create_node('approach_probe')
        holds, reports = [], []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: holds.append(json.loads(m.data)), 10)
        probe.create_subscription(String, CFG['report_topic'],
                                  lambda m: reports.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(probe)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # No costmap: the decision alone, as observe() leaves it on an approach.
            node.logic.approach_at, node.logic.gap = time.monotonic(), 2.5
            node.logic.at = [1.0, 2.0]
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        token = node.token
        node.destroy_node()
        probe.destroy_node()
        return holds, reports, token
    finally:
        rclpy.try_shutdown()


def test_node_enforcing_holds_with_its_token_and_reports():
    holds, reports, token = spin_node(True)
    assert holds and {h['op'] for h in holds} == {'hold'}
    assert {h['cause'] for h in holds} == {'approach'} and holds[0]['token'] == token
    assert reports[0] == {'gap_m': 2.5, 'at': [1.0, 2.0], 'enforce': True}


def test_node_log_only_reports_what_it_would_hold_on_and_never_holds():
    holds, reports, _ = spin_node(False)
    assert holds == []
    assert reports and reports[0]['enforce'] is False
    assert len(reports) <= 2.5 / CFG['repeat_s'] + 1   # lean: at most one per repeat_s
