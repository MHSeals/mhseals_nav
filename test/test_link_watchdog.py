"""Operator-link watchdog: hold-only, armed by the first heartbeat or by require_operator."""

import json
from pathlib import Path
import time

from mhseals_nav.link_watchdog import LinkWatchdog
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/link_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']


def test_no_operator_ever_seen_never_holds():
    dog = LinkWatchdog(CFG)
    for k in range(50):
        dog.command(k * 0.2)
        assert not dog.tick(k * 0.2)


def test_require_operator_holds_before_the_first_heartbeat():
    dog = LinkWatchdog(dict(CFG, require_operator=True))
    assert not dog.tick(0.0)               # no autonomy, nothing to stop
    dog.command(0.1)
    assert dog.tick(0.1)


def test_stale_heartbeat_holds_and_repeats_while_autonomy_flows():
    dog = LinkWatchdog(CFG)
    limit = CFG['max_age_s']
    dog.heartbeat(0.0)
    dog.command(limit)
    assert not dog.tick(limit)              # exactly at the limit is still fresh
    late = limit + 0.01
    dog.command(late)
    assert dog.tick(late)
    dog.command(late + 0.1)
    assert not dog.tick(late + 0.1)         # repeat_s throttles
    dog.command(late + CFG['repeat_s'])
    assert dog.tick(late + CFG['repeat_s'])  # a release while still stale is held again


def test_stale_heartbeat_without_autonomy_requests_nothing():
    dog = LinkWatchdog(CFG)
    dog.heartbeat(0.0)
    assert dog.reason(10.0)
    assert not dog.tick(10.0)


def test_heartbeat_back_never_releases():
    dog = LinkWatchdog(CFG)
    dog.heartbeat(0.0)
    dog.command(5.0)
    assert dog.tick(5.0)
    dog.heartbeat(5.5)
    dog.command(5.5)
    assert not dog.tick(5.5) and not dog.reason(5.5)


def test_node_holds_on_a_stale_heartbeat():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.link_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[Parameter('max_age_s', value=0.5)])
        probe = rclpy.create_node('link_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        beat = probe.create_publisher(String, CFG['heartbeat_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)
        start = time.monotonic()
        while time.monotonic() - start < 1.0:   # operator present: no hold
            beat.publish(String(data='tranquility'))
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        assert dog.logic.beat_at is not None and not got
        end = time.monotonic() + 4.0             # heartbeat stops, autonomy continues
        while time.monotonic() < end and len(got) < 2:
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        assert len(got) >= 2
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'operator_link'}
        assert {g['by'] for g in got} == {'link_watchdog'} and all(g['token'] for g in got)
    finally:
        rclpy.try_shutdown()
