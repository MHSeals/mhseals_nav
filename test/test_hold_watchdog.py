"""What counts as autonomy for the hold watchdogs: an auto command the mux can forward.

11-01 on the boat: every mux hold or authority change (a lapsed lease while the owner drove
by keyboard) brought link_watchdog and fence_watchdog holds 14-24 ms later. The TUI's
neutral zeros on /cmd_vel at the hold counted as autonomy while both conditions were bad.
"""

import json

from mhseals_nav.approach_guard import Guard
from mhseals_nav.fence_watchdog import FenceWatchdog
from mhseals_nav.link_watchdog import LinkWatchdog
from mhseals_nav.yaml_params import watchdog_config
import pytest


def config(name):
    return watchdog_config(name)


LINK = config('link_watchdog')
FENCE = config('fence_watchdog')
BOX = {'frame': 'map', 'points': [0.0, 0.0, 100.0, 0.0, 100.0, 50.0, 0.0, 50.0],
       'on_exit': 'alert'}
WINDOW = LINK['command_window']


def link(cfg=LINK):
    """No operator heartbeat (require_operator): the condition is bad from the start."""
    return LinkWatchdog(cfg, own_host='blastoise-odroid')


def fence():
    """Odometry and position missing: the condition is bad from the start."""
    return FenceWatchdog(FENCE, BOX)


def mux(dog, now, source):
    dog.mux(now, json.dumps({'input': source, 'holds': []}))


def test_a_restarted_watchdog_reuses_its_token_so_it_can_release_its_own_hold(
        tmp_path, monkeypatch):
    """The token lives as long as command_mux's held holds: the runtime dir, per domain."""
    import os
    from mhseals_nav.hold_watchdog import token
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '7')
    first = token('link_watchdog')                  # the watchdog that placed the hold
    assert token('link_watchdog') == first          # restarted: the same token releases it
    assert token('fence_watchdog') != first
    path = tmp_path / 'mhseals' / 'watchdog_link_watchdog_7.token'
    assert path.read_text() == first and os.stat(path).st_mode & 0o777 == 0o600
    assert sorted(p.suffix for p in path.parent.iterdir()) == ['.token', '.token']  # no temps
    monkeypatch.setenv('ROS_DOMAIN_ID', '8')
    assert token('link_watchdog') != first
    path.write_text('')                             # a torn or emptied file gets a new token
    monkeypatch.setenv('ROS_DOMAIN_ID', '7')
    assert token('link_watchdog') not in ('', first)


def test_every_config_names_the_mux_state():
    for name in ('approach_guard', 'costmap_watchdog', 'ekf_input_watchdog', 'fcu_watchdog',
                 'fence_watchdog', 'link_watchdog'):
        cfg = config(name)
        assert cfg['mux_state_topic'] == '/command_mux/state'
        assert cfg['mux_state_max_age'] > 1.0     # the mux republishes at least 1 Hz


@pytest.mark.parametrize('make', [link, fence])
@pytest.mark.parametrize('source', ['hold', 'manual', 'rc'])
def test_a_command_the_mux_does_not_forward_is_not_autonomy(make, source):
    dog = make()
    assert dog.reason(0.0)
    mux(dog, 0.0, source)
    for k in range(3):                 # the TUI's neutral repeats on /cmd_vel
        dog.command(0.01 + k * 0.001)
        assert not dog.tick(0.01 + k * 0.001)
    for t in (0.1, 0.3, WINDOW):
        assert not dog.tick(t)


@pytest.mark.parametrize('make', [link, fence])
def test_lease_lapse_during_keyboard_drive_does_not_cascade(make):
    """11:51:21: keyboard has the mux, the lease hold lands, zeros on /cmd_vel, then idle."""
    dog = make()
    mux(dog, 0.0, 'manual')
    dog.command(0.010)                 # neutral zeros, ahead of the lease hold
    assert not dog.tick(0.010)
    mux(dog, 0.012, 'hold')
    dog.command(0.014)
    assert not dog.tick(0.014)
    mux(dog, 0.2, 'idle')              # the hold cleared with U: nothing forwarded the zeros
    assert not dog.tick(0.2)
    mux(dog, 0.3, 'manual')            # keyboard again, and a second hold edge
    dog.command(0.31)
    mux(dog, 0.32, 'hold')
    assert not any(dog.tick(t) for t in (0.31, 0.32, 0.4, 0.6, 2.0))


@pytest.mark.parametrize('make', [link, fence])
@pytest.mark.parametrize('surge', [0.0, 1e-6, 0.5])
def test_autonomy_with_the_condition_bad_holds_at_once(make, surge):
    """The value never matters: a tiny command and a station-keeping zero both drive."""
    dog = make()
    mux(dog, 0.0, 'idle')
    dog.command(0.1)                   # the first auto command, before the mux says auto
    assert dog.tick(0.1)
    mux(dog, 0.12, 'auto')
    dog.command(1.2)
    assert dog.tick(1.2)               # held again after repeat


@pytest.mark.parametrize('make', [link, fence])
def test_mux_back_to_auto_counts_a_command_received_while_outranked(make):
    """Released, or keyboard timed out, with auto still flowing: hold on the state, not later."""
    dog = make()
    mux(dog, 0.0, 'hold')
    dog.command(0.1)
    assert not dog.tick(0.1)
    mux(dog, 0.2, 'auto')
    assert dog.tick(0.2)


@pytest.mark.parametrize('make', [link, fence])
def test_no_or_stale_or_unreadable_mux_state_counts_every_command(make):
    dog = make()
    dog.command(0.0)
    assert dog.tick(0.0)               # no mux state ever: as before
    dog = make()
    mux(dog, 0.0, 'hold')
    late = dog.cfg['mux_state_max_age'] + 0.01
    dog.command(late)
    assert dog.tick(late)              # a mux that stopped reporting decides nothing
    dog = make()
    dog.mux(0.0, 'not json')
    dog.command(0.1)
    assert dog.tick(0.1)
    dog = make()
    mux(dog, 0.0, 'something new')
    dog.command(0.1)
    assert dog.tick(0.1)


def test_keyboard_drive_with_the_link_bad_is_not_held():
    """By design link_watchdog covers autonomy only: the operator at the keys is the link."""
    dog = link()
    mux(dog, 0.0, 'manual')
    for k in range(100):
        t = k * 0.05
        if k % 20 == 0:
            mux(dog, t, 'manual')      # the 1 Hz republish
        assert not dog.tick(t)


# From the adversarial review of 327ba22 (deny side).


def test_missing_max_age_key_fails_closed():
    cfg = {k: v for k, v in LINK.items() if k != 'mux_state_max_age'}
    dog = link(cfg)
    mux(dog, 0.0, 'hold')
    dog.command(0.01)
    try:
        held = dog.tick(0.01)
    except KeyError:
        pytest.fail('missing mux_state_max_age raises KeyError in a callback (node dies)')
    assert held


def test_approach_seen_under_rc_still_holds_on_return_to_auto():
    """Before 327ba22 an approach under rc/manual with Nav2 still publishing left a hold."""
    g = Guard(dict(config('approach_guard'), enforce=True))
    mux(g, 0.0, 'rc')
    g.approach_at, g.gap = 0.0, 0.5     # an approach confirmed while the operator drives
    g.command(0.0)
    assert g.tick(0.0)                  # at confirmation, as before
    for k in range(1, 10):
        g.command(k * 0.1)
        assert not g.tick(k * 0.1)      # repeat throttles
    mux(g, 1.0, 'auto')                 # switch back to GUIDED: auto forwards
    g.command(1.0)
    assert g.tick(1.0), 'an approach 1 s earlier under rc leaves no hold on return'
    assert not g.owed
    mux(g, 1.1, 'rc')
    g.command(1.1)
    mux(g, 3.0, 'auto')
    g.command(3.0)
    assert not g.tick(3.0)              # owed once: sent, and no new approach since


@pytest.mark.parametrize('source', ['rc', 'manual'])
def test_no_hold_while_rc_or_manual_drives_without_an_approach(source):
    g = Guard(dict(config('approach_guard'), enforce=True))
    mux(g, 0.0, source)
    for k in range(20):
        g.command(k * 0.1)
        assert not g.tick(k * 0.1)
    mux(g, 2.0, 'auto')
    g.command(2.0)
    assert not g.tick(2.0) and not g.owed


def test_approach_confirmed_under_manual_holds_at_once():
    g = Guard(dict(config('approach_guard'), enforce=True))
    mux(g, 0.0, 'manual')
    g.command(0.0)
    assert not g.tick(0.0)
    g.approach_at, g.gap = 0.3, 0.5
    g.command(0.3)
    assert g.tick(0.3)


def test_approach_under_a_mux_hold_is_owed_and_never_expires():
    g = Guard(dict(config('approach_guard'), enforce=True))
    mux(g, 0.0, 'hold')
    g.approach_at, g.gap = 0.0, 0.5
    g.command(0.0)
    assert not g.tick(0.0) and g.owed   # nothing reaches the boat under the hold
    for t in (1.0, 2.0):                # the 1 Hz republish while still held
        mux(g, t, 'hold')
        assert not g.tick(t)
    mux(g, 60.0, 'auto')                # released a minute later, Nav2 still publishing
    g.command(60.0)
    assert g.tick(60.0)


def test_node_counts_every_command_with_two_mux_state_publishers():
    """A bag replay or a stray sim mux beside the real one could otherwise mask a hold."""
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import spin
    require_isolated_domain()
    from mhseals_nav.link_watchdog import node_class
    rclpy.init()
    try:
        # Its own topic: these publishers must not linger in discovery for the next test.
        topic = '/two_mux_probe/state'
        dog = node_class()(parameter_overrides=[Parameter('mux_state_topic', value=topic)])
        probe = rclpy.create_node('mux_probe')
        got = []
        probe.create_subscription(String, LINK['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, LINK['command_topic'], 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        states = [probe.create_publisher(String, topic, latched)
                  for _ in range(2)]
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def feed():
            for state in states:
                state.publish(String(data=json.dumps({'input': 'hold'})))
            cmd.publish(Twist())

        spin(ex, feed, 3.0, until=lambda: got)
        assert got and got[0]['cause'] == 'operator_link'
    finally:
        rclpy.try_shutdown()


def test_node_ignores_zeros_under_a_mux_hold_and_holds_under_auto():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import spin
    require_isolated_domain()
    from mhseals_nav.link_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()()
        probe = rclpy.create_node('mux_probe')
        got = []
        probe.create_subscription(String, LINK['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, LINK['command_topic'], 10)
        state = probe.create_publisher(String, LINK['mux_state_topic'], QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def feed(source):
            def go():
                state.publish(String(data=json.dumps({'input': source})))
                cmd.publish(Twist())
            return go

        # The state first: a command ahead of any mux state counts (unknown), as it should.
        # One mux state publisher on the graph (an earlier test's may linger in discovery).
        spin(ex, lambda: state.publish(String(data=json.dumps({'input': 'hold'}))), 5.0,
             until=lambda: dog.count_publishers(
            LINK['mux_state_topic']) == 1 and dog.logic.mux_input == 'hold')
        assert dog.logic.mux_input == 'hold' and dog.logic.reason(0.0)
        spin(ex, feed('hold'), 1.5)
        assert got == []                     # zeros while the mux holds: no cascade
        spin(ex, feed('auto'), 2.0, until=lambda: got)
        assert got and got[0]['cause'] == 'operator_link'
    finally:
        rclpy.try_shutdown()

