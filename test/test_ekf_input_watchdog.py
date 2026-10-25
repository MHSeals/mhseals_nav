"""EKF-input watchdog: /imu/raw, /gps/fix and /odom/mavros must be fresh (sim fault F6)."""

import json
from pathlib import Path
import time

from mhseals_nav.ekf_input_watchdog import EkfInputWatchdog, INPUTS
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/ekf_input_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']


def feed(dog, now, skip=None):
    for name in INPUTS:
        if name != skip:
            dog.receive(name, now)
    dog.command(now)


def test_fresh_inputs_request_nothing():
    dog = EkfInputWatchdog(CFG)
    for k in range(30):
        feed(dog, k * 0.1)
        assert not dog.tick(k * 0.1) and not dog.reason(k * 0.1)


def test_never_seen_inputs_hold_only_under_autonomy():
    dog = EkfInputWatchdog(CFG)
    assert not dog.tick(0.0)
    dog.command(0.0)
    assert dog.tick(0.0)
    assert dog.stale_inputs(0.0) == [CFG[f'{n}_topic'] for n in INPUTS]


@pytest.mark.parametrize('silent', INPUTS)
def test_each_input_stale_then_fresh(silent):
    """Fresh up to its limit, held just past it, and no request once it is fresh again."""
    limit = CFG[f'max_{silent}_age_s']
    dog = EkfInputWatchdog(CFG)
    feed(dog, 0.0)
    feed(dog, limit, skip=silent)
    assert not dog.tick(limit)                                  # fresh at the limit
    stale = limit + 0.01
    feed(dog, stale, skip=silent)
    assert dog.tick(stale)                                      # stale: hold
    assert dog.stale_inputs(stale) == [CFG[f'{silent}_topic']]
    feed(dog, stale + 0.5, skip=silent)
    assert not dog.tick(stale + 0.5)                            # repeat_s throttles
    again = stale + CFG['repeat_s'] + 0.01
    feed(dog, again, skip=silent)
    assert dog.tick(again)                                      # re-held while still stale
    back = again + 0.1
    feed(dog, back)
    assert not dog.tick(back) and not dog.reason(back)          # fresh: no request, no release


def test_no_autonomy_no_hold():
    dog = EkfInputWatchdog(CFG)
    feed(dog, 0.0)
    assert not dog.tick(CFG['command_window_s'] + 5.0)          # stale, but nothing drives


def test_node_holds_when_relays_die_while_mavros_lives():
    """S4-relay: MAVROS keeps publishing while its relays and adapter stop.

    fcu_watchdog stays quiet by design; ekf_input_watchdog must hold, repeatedly, and only
    ever hold.
    """
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import flip_twice, spin
    require_isolated_domain()
    from mhseals_nav import ekf_input_watchdog, fcu_watchdog
    rclpy.init()
    try:
        inputs = ekf_input_watchdog.node_class()()
        fcu = fcu_watchdog.node_class()()
        fcu_cfg = fcu.cfg
        probe = rclpy.create_node('ekf_input_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        mavros = (probe.create_publisher(State, fcu_cfg['state_topic'], 10),
                  probe.create_publisher(Imu, fcu_cfg['imu_topic'], 10),
                  probe.create_publisher(NavSatFix, fcu_cfg['gps_topic'], 10))
        relays = (probe.create_publisher(Imu, CFG['imu_topic'], 10),
                  probe.create_publisher(NavSatFix, CFG['gps_topic'], 10),
                  probe.create_publisher(Odometry, CFG['odom_topic'], 10))
        ex = SingleThreadedExecutor()
        for node in (inputs, fcu, probe):
            ex.add_node(node)

        def publish(relays_alive, autonomy=True):
            def tick():
                state, imu, gps = mavros
                state.publish(State(connected=True))
                imu.publish(Imu())
                gps.publish(NavSatFix())
                if relays_alive:
                    relays[0].publish(Imu())
                    relays[1].publish(NavSatFix())
                    relays[2].publish(Odometry())
                if autonomy:
                    cmd.publish(Twist())
            return tick

        spin(ex, publish(True, autonomy=False), 0.5)  # discovery, before autonomy starts
        spin(ex, publish(True), 1.0)                 # all alive under autonomy: no hold
        assert not got and not inputs.logic.reason(time.monotonic())
        flip_twice(ex, got, publish(False), publish(True))
        assert {g['cause'] for g in got} == {CFG['cause']}
        assert {g['by'] for g in got} == {'ekf_input_watchdog'} and all(g['token'] for g in got)
        assert not fcu.logic.reason(time.monotonic())  # MAVROS itself was alive throughout
    finally:
        rclpy.try_shutdown()
