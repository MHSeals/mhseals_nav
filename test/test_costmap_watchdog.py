"""Stale-costmap watchdog: hold-only, only while autonomy commands flow."""

import json
from pathlib import Path
import time

import pytest
import yaml

from mhseals_nav.costmap_watchdog import Watchdog

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/costmap_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']


def test_missing_costmap_holds_only_under_autonomy():
    dog = Watchdog(CFG)
    assert not dog.tick(0.0)               # no costmap, no autonomy: nothing to stop
    dog.command(0.1)
    assert dog.tick(0.1)                   # never had a costmap
    assert not dog.tick(0.2)               # repeat_s throttles
    dog.command(1.2)
    assert dog.tick(1.2)                   # re-held so a release while stale cannot resume


def test_fresh_costmap_requests_nothing_and_staleness_holds():
    dog = Watchdog(CFG)
    for k in range(20):
        now = k * 0.2
        dog.costmap(now, 0.05)
        dog.command(now)
        assert not dog.tick(now)
    dog.command(3.8 + CFG['max_age_s'] + 0.01)
    assert dog.tick(3.8 + CFG['max_age_s'] + 0.01)


def test_old_stamp_is_not_fresh():
    dog = Watchdog(CFG)
    dog.costmap(0.0, CFG['max_stamp_age_s'] + 1.0)   # a latched map replayed late
    dog.command(0.0)
    assert dog.tick(0.0)


def test_fresh_again_never_releases():
    dog = Watchdog(CFG)
    dog.command(0.0)
    assert dog.tick(0.0)
    dog.costmap(0.5, 0.0)
    dog.command(0.5)
    assert not dog.tick(0.5)               # no request at all: in particular no release


def test_node_publishes_hold_requests_only():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.costmap_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()()
        probe = rclpy.create_node('watchdog_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and len(got) < 2:
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        assert len(got) >= 2
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {CFG['cause']}
        assert all(g['token'] for g in got)
    finally:
        rclpy.try_shutdown()
