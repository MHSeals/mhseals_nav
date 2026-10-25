"""Geofence watchdog: outside, or near the edge heading outward, holds; so does no fence."""

import json
import math
from pathlib import Path
import time

from mhseals_nav.fence_watchdog import FenceWatchdog, load_fence, polygon, project
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/fence_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']
BOX = [0.0, 0.0, 100.0, 0.0, 100.0, 50.0, 0.0, 50.0]      # map metres, x east, y north
MAP = dict(CFG, fence_frame='map', fence_points=BOX)


def at(dog, now, x, y, yaw=0.0, vx=0.0):
    dog.position(now, x, y)
    dog.motion(now, yaw, vx, 0.0)
    dog.command(now)
    return dog.tick(now)


def test_well_inside_requests_nothing_whatever_the_heading():
    dog = FenceWatchdog(MAP)
    for k, yaw in enumerate((0.0, 1.5, 3.1, -1.5)):
        assert not at(dog, k * 2.0, 50.0, 25.0, yaw, 1.0)


def test_outside_holds_only_under_autonomy():
    dog = FenceWatchdog(MAP)
    dog.position(0.0, 120.0, 25.0)
    dog.motion(0.0, math.pi, 0.0, 0.0)       # facing back in, still outside
    assert dog.reason(0.0) == 'outside the fence'
    assert not dog.tick(0.0)
    dog.command(0.0)
    assert dog.tick(0.0)


def test_near_the_edge_holds_only_heading_outward():
    dog = FenceWatchdog(MAP)
    edge = 100.0 - CFG['margin_m'] / 2      # inside the margin of the east edge
    assert at(dog, 0.0, edge, 25.0, 0.0, 0.0)                    # facing east, still
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, edge, 25.0, math.pi, 0.5)            # facing and moving west
    assert not at(dog, 2.0, edge, 25.0, math.pi / 2, 0.5)        # along the edge
    assert at(dog, 4.0, edge, 25.0, math.pi, -0.5)               # backing out east


def test_outward_angle_and_margin_bounds():
    dog = FenceWatchdog(MAP)
    inside = math.radians(CFG['outward_angle_deg']) - 0.05
    edge = 100.0 - CFG['margin_m'] + 0.1
    assert at(dog, 0.0, edge, 25.0, inside, 0.0)
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, edge, 25.0, inside + 0.1, 0.0)
    assert not at(dog, 2.0, 100.0 - CFG['margin_m'] - 0.1, 25.0, 0.0, 1.0)


def test_near_a_corner_both_edges_count():
    for yaw in (0.0, math.pi / 4, math.pi / 2):                   # east, NE, north
        dog = FenceWatchdog(MAP)
        assert at(dog, 0.0, 98.5, 47.0, yaw, 0.0)                 # 1.5 m from E, 3 m from N
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, 98.5, 47.0, -3 * math.pi / 4, 0.0)


def test_no_valid_fence_holds():
    for points in ([], [0.0, 0.0, 1.0, 1.0], BOX[:-1], BOX[:-2] + [0.0, float('nan')],
                   BOX[:-1] + ['x'], {'a': 1}):
        dog = FenceWatchdog(dict(MAP, fence_points=points))
        assert dog.poly is None
        assert at(dog, 0.0, 50.0, 25.0)
    assert polygon('utm', BOX) is None
    dog = FenceWatchdog({k: v for k, v in CFG.items() if k != 'fence_points'})
    assert at(dog, 0.0, 42.0, -71.0)


def test_stale_position_or_odometry_holds():
    dog = FenceWatchdog(MAP)
    dog.motion(0.0, 0.0, 0.0, 0.0)
    dog.command(0.0)
    assert dog.reason(0.0) == 'position missing or stale' and dog.tick(0.0)
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, 50.0, 25.0)
    late = CFG['max_age_s'] + 0.01
    dog.position(late, 50.0, 25.0)
    dog.command(late)
    assert dog.reason(late) == 'odometry missing or stale' and dog.tick(late)


def test_lat_lon_fence():
    lat0, lon0 = 42.0, -71.0
    d = 0.001                                # ~111 m north, ~83 m east
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=[
        lat0, lon0, lat0, lon0 + d, lat0 + d, lon0 + d, lat0 + d, lon0]))
    east, north = project((lat0, lon0), lat0 + d, lon0 + d)
    assert east == pytest.approx(82.7, abs=0.2) and north == pytest.approx(111.2, abs=0.2)
    assert not at(dog, 0.0, lat0 + d / 2, lon0 + d / 2, 0.0, 1.0)
    assert at(dog, 2.0, lat0 + d / 2, lon0 + d + 1e-5, math.pi, 0.0)   # 0.8 m outside east
    dog = FenceWatchdog(dog.cfg)
    assert at(dog, 0.0, lat0 + d / 2, lon0 + d - 2e-5, 0.0, 0.0)      # 1.7 m in, facing east


def test_fence_file_overrides_the_package_fence(tmp_path):
    day = tmp_path / 'fence.yaml'
    day.write_text(yaml.safe_dump({'fence_frame': 'map', 'fence_points': BOX}))
    cfg = load_fence(dict(CFG, fence_file=str(day), fence_points=[1.0]))
    assert (cfg['fence_frame'], cfg['fence_points']) == ('map', BOX)
    for text in ('- 1\n', ': :\n', 'fence_frame: map\n'):
        day.write_text(text)
        assert FenceWatchdog(load_fence(dict(CFG, fence_file=str(day)))).poly is None
    assert load_fence(dict(CFG, fence_file=str(tmp_path / 'missing.yaml'))) == \
        dict(CFG, fence_file=str(tmp_path / 'missing.yaml'))


def test_back_inside_never_releases():
    dog = FenceWatchdog(MAP)
    assert at(dog, 0.0, 120.0, 25.0)
    assert not at(dog, 0.5, 50.0, 25.0)
    assert not dog.reason(0.5)


def test_node_holds_outside_a_map_fence():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX)])
        probe = rclpy.create_node('fence_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def drive(x, seconds, until=None, autonomy=True):
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, 25.0
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = 1.0
            end = time.monotonic() + seconds
            while time.monotonic() < end and not (until and until()):
                odom.publish(msg)
                if autonomy:
                    cmd.publish(Twist())
                ex.spin_once(timeout_sec=0.05)

        drive(50.0, 0.5, autonomy=False)        # odometry before autonomy starts
        drive(50.0, 1.0)
        assert dog.logic.poly is not None and dog.logic.p == (50.0, 25.0) and not got
        drive(101.0, 3.0, until=lambda: len(got) >= 2)
        assert len(got) >= 2
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'geofence'}
        assert {g['by'] for g in got} == {'fence_watchdog'} and all(g['token'] for g in got)
    finally:
        rclpy.try_shutdown()
