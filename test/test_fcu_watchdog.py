"""FCU-link watchdog: MAVROS state, IMU and raw GPS must be fresh and connected."""

import json
from pathlib import Path
import time

from mhseals_nav.fcu_watchdog import FcuWatchdog
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/fcu_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']


def healthy(dog, now):
    dog.state(now, True)
    dog.imu(now)
    dog.gps(now)
    dog.command(now)


def test_healthy_link_requests_nothing():
    dog = FcuWatchdog(CFG)
    for k in range(30):
        healthy(dog, k * 0.1)
        assert not dog.tick(k * 0.1)


def test_never_seen_mavros_holds_only_under_autonomy():
    dog = FcuWatchdog(CFG)
    assert not dog.tick(0.0)
    dog.command(0.0)
    assert dog.tick(0.0)
    assert dog.reason(0.0) == 'MAVROS state missing or stale'


def test_disconnected_holds():
    dog = FcuWatchdog(CFG)
    healthy(dog, 0.0)
    dog.state(0.1, False)
    dog.command(0.1)
    assert dog.tick(0.1)
    assert dog.reason(0.1) == 'MAVROS reports the FCU disconnected'


@pytest.mark.parametrize('silent, limit, name', [
    ('imu', 'max_imu_age_s', 'IMU'), ('gps', 'max_gps_age_s', 'GPS'),
    ('state', 'max_state_age_s', 'state')])
def test_each_silent_input_holds_past_its_limit(silent, limit, name):
    dog = FcuWatchdog(CFG)
    healthy(dog, 0.0)
    for now, hold in ((CFG[limit], False), (CFG[limit] + 0.01, True)):
        for feed in ('imu', 'gps'):
            if feed != silent:
                getattr(dog, feed)(now)
        if silent != 'state':
            dog.state(now, True)
        dog.command(now)
        assert dog.tick(now) == hold
    assert name in dog.reason(now)


def test_back_again_never_releases():
    dog = FcuWatchdog(CFG)
    dog.command(0.0)
    assert dog.tick(0.0)
    healthy(dog, 0.5)
    assert not dog.tick(0.5) and not dog.reason(0.5)


def test_node_holds_when_mavros_goes_silent():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from mavros_msgs.msg import State
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import flip_twice
    require_isolated_domain()
    from mhseals_nav.fcu_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()()
        probe = rclpy.create_node('fcu_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        state = probe.create_publisher(State, CFG['state_topic'], 10)
        imu = probe.create_publisher(Imu, CFG['imu_topic'], 10)
        gps = probe.create_publisher(NavSatFix, CFG['gps_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)
        start = time.monotonic()
        while time.monotonic() - start < 1.5:   # MAVROS alive: no hold once it is seen
            state.publish(State(connected=True))
            imu.publish(Imu())
            gps.publish(NavSatFix())
            if time.monotonic() - start > 0.5:
                cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.02)
        assert dog.logic.autonomy(time.monotonic()) and not dog.logic.reason(time.monotonic())
        assert not got
        end = time.monotonic() + 4.0             # MAVROS dies, autonomy continues
        while time.monotonic() < end and len(got) < 2:
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        assert len(got) >= 2
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'fcu_link'}
        assert {g['by'] for g in got} == {'fcu_watchdog'} and all(g['token'] for g in got)

        def mavros(connected):
            def feed():
                state.publish(State(connected=connected))
                imu.publish(Imu())
                gps.publish(NavSatFix())
                cmd.publish(Twist())
            return feed

        flip_twice(ex, got, mavros(False), mavros(True))
    finally:
        rclpy.try_shutdown()
