"""Geofence watchdog: outside, or near the edge heading outward, holds; so does no fence."""

import itertools
import json
import math
from pathlib import Path
import time

from mhseals_nav.fence_watchdog import FenceWatchdog, load_fence, outline, polygon, project
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


def test_near_edge_reason_does_not_change_with_the_distance():
    """The node logs on a reason change: a distance in it would log every tick."""
    dog = FenceWatchdog(MAP)
    reasons = set()
    for k, x in enumerate((96.0, 97.0, 98.0)):
        assert at(dog, k * 1.0, x, 25.0)
        reasons.add(dog.reason(k * 1.0))
    assert reasons == {'within the margin heading outward'}
    assert dog.describe(dog.reason(2.0)) == 'within the margin heading outward (2.0 m)'
    assert dog.describe('outside the fence') == 'outside the fence'


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


def test_unreadable_fence_file_holds(tmp_path, monkeypatch):
    """Even with a valid fence in the parameters: the day's file was meant to replace it."""
    day = tmp_path / 'fence.yaml'
    day.write_bytes(b'fence_frame: map\nfence_points: [\xff\xfe]\n')   # not UTF-8
    dog = FenceWatchdog(load_fence(dict(MAP, fence_file=str(day))))
    assert dog.poly is None and at(dog, 0.0, 50.0, 25.0)
    day.write_text(yaml.safe_dump({'fence_frame': 'map', 'fence_points': BOX}))
    day.chmod(0)                       # root still reads it, so also refuse the read itself

    def refuse(*_, **__):
        raise PermissionError(13, 'Permission denied', str(day))
    monkeypatch.setattr(Path, 'read_text', refuse)
    dog = FenceWatchdog(load_fence(dict(MAP, fence_file=str(day))))
    assert dog.poly is None and at(dog, 0.0, 50.0, 25.0)


STATE_KEYS = {'valid', 'frame', 'points', 'margin_m', 'outward_angle_deg', 'outward_speed_mps',
              'reason', 'error', 'stamp_ns'}


def test_state_of_a_map_fence_is_its_points():
    dog = FenceWatchdog(MAP)
    state = dog.state(0.0, 7)
    assert set(state) == STATE_KEYS
    assert state['valid'] and state['frame'] == 'map'
    assert state['points'] == BOX and state['stamp_ns'] == 7 and not state['error']
    assert state['margin_m'] == CFG['margin_m']
    assert state['reason'] == 'position missing or stale'
    at(dog, 0.0, 99.0, 25.0)
    assert dog.state(0.0)['reason'] == 'within the margin heading outward'
    assert json.loads(json.dumps(state)) == state


def test_state_without_a_fence_is_invalid():
    state = FenceWatchdog(dict(MAP, fence_points=[])).state(0.0)
    assert set(state) == STATE_KEYS
    assert not state['valid'] and state['points'] is None and state['error']


def test_state_of_a_lat_lon_fence_is_its_raw_lat_lon_whatever_the_position():
    """No map copy: the TUI checks goals in lat/lon (toLL), so a rotated map cannot shift it."""
    points = [42.0, -71.0, 42.0, -70.999, 42.001, -70.999, 42.001, -71.0]
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=points))
    for state in (dog.state(0.0), dog.state(100.0)):         # no fix yet, then a stale one
        assert set(state) == STATE_KEYS and state['valid'] and state['frame'] == 'latlon'
        assert state['points'] == points
        dog.position(0.0, 42.0005, -70.9995)


LATLON = [42.0, -71.0, 42.0, -70.999, 42.001, -70.999, 42.001, -71.0]


def test_anchor_moves_a_map_fence_as_the_start_placed_it():
    dog = FenceWatchdog(MAP)
    assert dog.anchor(0.0) == (False, 'no boat position (missing or older than max_age_s)')
    at(dog, 0.0, 10.0, 20.0)
    ok, message = dog.anchor(0.0)
    assert ok and 'fence anchored at 10, 20 (map)' == message
    assert dog.cfg['fence_points'] == [10.0, 20.0, 110.0, 20.0, 110.0, 70.0, 10.0, 70.0]
    assert dog.state(0.0)['points'] == dog.cfg['fence_points']
    assert MAP['fence_points'] == BOX                       # the package config is untouched
    assert dog.reason(0.0) == 'on the fence'                # BOX's corner is the origin
    at(dog, 0.5, 105.0, 50.0)                               # outside the old box, in the new
    assert dog.reason(0.5) == ''
    at(dog, 1.0, 5.0, 50.0)                                 # in the old box, out of the new
    assert dog.reason(1.0) == 'outside the fence'
    at(dog, 1.5, 15.0, 25.0)                                # anchoring again is relative too
    assert dog.anchor(1.5)[0]
    assert dog.cfg['fence_points'][:2] == [15.0, 25.0]


def test_anchor_refuses_without_a_fresh_position_or_a_fence():
    dog = FenceWatchdog(MAP)
    at(dog, 0.0, 10.0, 20.0)
    ok, why = dog.anchor(CFG['max_age_s'] + 0.01)
    assert not ok and 'no boat position' in why and dog.cfg['fence_points'] == BOX
    dog = FenceWatchdog(dict(MAP, fence_points=[]))
    at(dog, 0.0, 10.0, 20.0)
    assert dog.anchor(0.0) == (False, 'no valid fence configured')


def test_anchor_refuses_while_moving_or_without_odometry():
    dog = FenceWatchdog(MAP)
    dog.position(0.0, 10.0, 20.0)
    ok, why = dog.anchor(0.0)
    assert not ok and 'no odometry' in why and dog.cfg['fence_points'] == BOX
    at(dog, 0.0, 10.0, 20.0, 1.0, CFG['anchor_max_speed_mps'] + 0.1)
    ok, why = dog.anchor(0.0)
    assert not ok and 'moving' in why and dog.cfg['fence_points'] == BOX
    at(dog, 0.5, 10.0, 20.0, 1.0, CFG['anchor_max_speed_mps'] / 2)
    assert dog.anchor(0.5)[0]


def test_anchor_centres_a_lat_lon_fence_on_the_boat():
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=LATLON))
    boat = (42.1, -71.2)
    at(dog, 0.0, *boat)
    assert dog.reason(0.0) == 'outside the fence'
    assert dog.anchor(0.0)[0]
    points = dog.cfg['fence_points']
    assert sum(points[::2]) / 4 == pytest.approx(boat[0])
    assert sum(points[1::2]) / 4 == pytest.approx(boat[1])
    assert [b - a for a, b in zip(points, points[2:])] == pytest.approx(
        [b - a for a, b in zip(LATLON, LATLON[2:])])          # same shape
    assert dog.state(0.0)['frame'] == 'latlon' and dog.state(0.0)['points'] == points
    at(dog, 0.5, *boat)
    assert dog.reason(0.5) == ''


def test_outline_of_a_map_fence_is_its_closed_points():
    assert outline('map', BOX) == [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0),
                                   (0.0, 0.0)]
    assert outline('map', BOX[:-1]) is None and outline('utm', BOX) is None
    dog = FenceWatchdog(MAP)
    assert dog.outline(0.0) == outline('map', BOX)          # no position needed
    at(dog, 0.0, 10.0, 20.0)
    dog.anchor(0.0)
    assert dog.outline(100.0)[0] == (10.0, 20.0)


def test_outline_of_a_lat_lon_fence_is_placed_through_the_boat():
    assert outline('latlon', LATLON) is None                # nowhere to place it yet
    line = outline('latlon', LATLON, boat_raw=(42.0, -71.0), boat_map=(5.0, -3.0))
    assert len(line) == 5 and line[0] == line[-1] == (5.0, -3.0)
    east, north = project((42.0, -71.0), 42.001, -70.999)
    assert line[2] == pytest.approx((5.0 + east, -3.0 + north))
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=LATLON))
    dog.position(0.0, 42.0, -71.0)
    assert dog.outline(0.0) is None                         # no map pose yet
    dog.map_position(0.0, 5.0, -3.0)
    assert dog.outline(0.0) == line
    assert dog.outline(CFG['max_age_s'] + 0.01) is None     # stale


def test_back_inside_holds_no_more():
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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import flip_twice
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

        def odometry(x):
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, 25.0
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = 1.0
            return msg

        def drive(x, seconds, until=None, autonomy=True):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not (until and until()):
                odom.publish(odometry(x))
                if autonomy:
                    cmd.publish(Twist())
                ex.spin_once(timeout_sec=0.05)

        drive(50.0, 0.5, autonomy=False)        # odometry before autonomy starts
        drive(50.0, 1.0)
        assert dog.logic.poly is not None and dog.logic.p == (50.0, 25.0) and not got
        drive(101.0, 3.0, until=lambda: len(got) >= 2)
        assert len(got) >= 2

        def at_x(xs):
            xs = iter(xs)

            def feed():
                odom.publish(odometry(next(xs)))
                cmd.publish(Twist())
            return feed

        # Near the east edge heading east, the distance changing every message.
        near = at_x(97.0 + 0.01 * (k % 100) for k in itertools.count())
        flip_twice(ex, got, near, at_x(itertools.repeat(50.0)))
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'geofence'}
        assert {g['by'] for g in got} == {'fence_watchdog'} and all(g['token'] for g in got)

        # The fence it enforces is latched on fence_topic: a late subscriber gets it.
        fences = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        drive(50.0, 1.2)
        probe.create_subscription(String, CFG['fence_topic'],
                                  lambda m: fences.append(json.loads(m.data)), latched)
        drive(50.0, 1.0, until=lambda: fences)
        assert fences and fences[0]['valid'] and fences[0]['points'] == BOX

        # A failing publish never stops the holds.
        def broken(*_, **__):
            raise ValueError('boom')
        dog.logic.state = broken
        got.clear()
        drive(101.0, 3.0, until=lambda: len(got) >= 2)
        assert len(got) >= 2
    finally:
        rclpy.try_shutdown()


def test_node_anchor_service_moves_the_latched_marker():
    rclpy = pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX)])
        probe = rclpy.create_node('fence_anchor_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        markers = []
        probe.create_subscription(Marker, CFG['marker_topic'], markers.append, latched)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        anchor = probe.create_client(Trigger, CFG['anchor_service'])
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def spin(seconds, until):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                ex.spin_once(timeout_sec=0.05)

        def call():
            future = anchor.call_async(Trigger.Request())
            spin(3.0, future.done)
            return future.result()

        spin(3.0, lambda: markers)
        first = markers[-1]
        assert first.type == Marker.LINE_STRIP and first.header.frame_id == 'map'
        assert [(p.x, p.y) for p in first.points] == outline('map', BOX)
        assert anchor.wait_for_service(timeout_sec=3.0)
        refused = call()
        assert not refused.success and 'no boat position' in refused.message

        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = 10.0, 20.0
        msg.pose.pose.orientation.w = 1.0
        odom.publish(msg)
        spin(1.0, lambda: dog.logic.p == (10.0, 20.0))
        markers.clear()
        done = call()
        assert done.success and 'anchored' in done.message
        spin(2.0, lambda: markers)
        assert [(p.x, p.y) for p in markers[-1].points][0] == (10.0, 20.0)
    finally:
        rclpy.try_shutdown()
