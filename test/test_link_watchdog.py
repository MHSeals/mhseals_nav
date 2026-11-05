"""Operator-link watchdog: hold-only, armed by the first heartbeat or by require_operator."""

import json
import os

from mhseals_nav.link_watchdog import host_name, LinkWatchdog
from mhseals_nav.yaml_params import watchdog_config
import pytest

BOAT_CFG = watchdog_config('link_watchdog')
OPERATOR = 'opstation'
# Literal hosts here; the station token has its own tests below (never the real home file).
CFG = dict(BOAT_CFG, operator_hosts=[OPERATOR])


def watchdog(**over):
    return LinkWatchdog(dict(CFG, **over), own_host='blastoise')


def test_boat_config_requires_an_operator_and_follows_the_station():
    assert BOAT_CFG['require_operator'] and BOAT_CFG['operator_hosts'] == ['station']
    assert BOAT_CFG['station_file'] == '~/.config/mhseals/station.yaml'


def station_dog(tmp_path, hosts=('station',)):
    logs = []
    dog = LinkWatchdog(dict(CFG, operator_hosts=list(hosts),
                            station_file=str(tmp_path / 'station.yaml')),
                       own_host='blastoise', log=lambda msg, bad: logs.append((bad, msg)))
    return dog, tmp_path / 'station.yaml', logs


def take(path, operator, mtime):
    path.write_text(f'operator: {operator}\nsince: 2026-11-01T20:00:00\n')
    os.utime(path, ns=(mtime, mtime))     # an mtime change, however fast the writes


def test_no_station_file_counts_nobody_holds_with_the_reason_and_warns_once(tmp_path):
    dog, _, logs = station_dog(tmp_path)
    for host in ('laptop-a', 'laptop-b', 'Blastoise.local', ''):
        assert not dog.heartbeat(0.0, host)
    assert dog.reason(0.0) == 'no station taken: hw.sh station take'
    dog.command(0.1)
    assert dog.tick(0.1)
    assert [bad for bad, _ in logs] == [True] and 'no station file' in logs[0][1]


def test_without_require_operator_no_station_never_holds(tmp_path):
    dog, _, _ = station_dog(tmp_path)
    dog.cfg['require_operator'] = False
    dog.command(0.1)
    assert dog.reason(0.1) == '' and not dog.tick(0.1)


def test_the_station_file_names_the_operator_and_is_re_read_on_change(tmp_path):
    dog, path, logs = station_dog(tmp_path)
    take(path, 'Laptop-A.local', 1)
    assert dog.heartbeat(0.0, 'laptop-a') and not dog.heartbeat(0.0, 'laptop-b')
    take(path, 'laptop-b', 2)              # another computer took the station
    assert dog.heartbeat(1.0, 'laptop-b') and not dog.heartbeat(1.0, 'laptop-a')
    assert [msg for bad, msg in logs if not bad] == ['operator station: laptop-a',
                                                     'operator station: laptop-b']
    path.unlink()                          # released: the last beat goes stale, then holds
    assert not dog.heartbeat(2.0, 'laptop-b')
    assert logs[-1][0] and 'no station file' in logs[-1][1]
    assert dog.reason(2.0 + dog.cfg['max_age'] + 1) == 'no station taken: hw.sh station take'


@pytest.mark.parametrize('text', ['operator:\n', '[a, b]\n', 'operator: [\n', 'since: x\n'])
def test_a_station_file_without_an_operator_counts_nobody(tmp_path, text):
    dog, path, logs = station_dog(tmp_path)
    path.write_text(text)
    assert not dog.heartbeat(0.0, 'laptop-a') and not dog.heartbeat(0.0, 'blastoise')
    assert len(logs) == 1 and logs[0][0]


def test_a_station_file_of_bad_bytes_counts_nobody_without_crashing(tmp_path):
    dog, path, logs = station_dog(tmp_path)
    path.write_bytes(b'operator: \xff\xfe\n')
    assert not dog.heartbeat(0.0, 'laptop-a') and len(logs) == 1 and logs[0][0]


def test_a_write_in_the_same_mtime_tick_as_the_truncate_is_read(tmp_path):
    dog, path, _ = station_dog(tmp_path)
    path.write_text('')
    os.utime(path, ns=(1, 1))
    assert not dog.heartbeat(0.0, 'laptop-b')  # empty: nobody
    take(path, 'laptop-a', 1)              # same mtime, new size
    assert not dog.heartbeat(0.5, 'laptop-b') and dog.heartbeat(0.5, 'laptop-a')


def test_literal_hosts_count_beside_the_station(tmp_path):
    dog, path, _ = station_dog(tmp_path, hosts=('station', 'Backup-Laptop'))
    take(path, 'laptop-a', 1)
    assert dog.heartbeat(0.0, 'backup-laptop') and dog.heartbeat(0.0, 'laptop-a')
    assert not dog.heartbeat(0.0, 'laptop-b')
    take(path, 'blastoise', 2)             # a station naming the boat never counts the boat
    assert not dog.heartbeat(1.0, 'blastoise') and dog.heartbeat(1.0, 'backup-laptop')


def test_no_operator_ever_seen_never_holds_unless_required():
    dog = watchdog(require_operator=False)
    for k in range(50):
        dog.command(k * 0.2)
        assert not dog.tick(k * 0.2)


def test_require_operator_holds_before_the_first_heartbeat():
    dog = watchdog()
    assert not dog.tick(0.0)               # no autonomy, nothing to stop
    dog.command(0.1)
    assert dog.tick(0.1)


def test_only_operator_hosts_count_and_never_the_boat_itself():
    dog = watchdog(operator_hosts=[OPERATOR, 'blastoise'])
    for host in ('blastoise', 'Blastoise.local', 'squirtle', '', OPERATOR + '-x'):
        assert not dog.heartbeat(0.0, host)
    assert dog.beat_at is None
    dog.command(0.0)
    assert dog.tick(0.0)                   # a TUI on the boat does not arm or satisfy it
    assert dog.heartbeat(0.1, 'OpStation.local')
    assert not dog.reason(0.1)
    dog.heartbeat(5.0, 'blastoise')        # the boat's own beats do not keep it fresh
    assert dog.reason(5.0)


def test_stale_heartbeat_holds_and_repeats_while_autonomy_flows():
    dog = watchdog()
    limit = CFG['max_age']
    dog.heartbeat(0.0, OPERATOR)
    dog.command(limit)
    assert not dog.tick(limit)              # exactly at the limit is still fresh
    late = limit + 0.01
    dog.command(late)
    assert dog.tick(late)
    dog.command(late + 0.1)
    assert not dog.tick(late + 0.1)         # repeat throttles
    dog.command(late + CFG['repeat'])
    assert dog.tick(late + CFG['repeat'])  # a release while still stale is held again


def test_stale_heartbeat_without_autonomy_requests_nothing():
    dog = LinkWatchdog(CFG)
    dog.heartbeat(0.0, OPERATOR)
    assert dog.reason(10.0)
    assert not dog.tick(10.0)


def test_heartbeat_back_holds_no_more_and_releases_nothing_at_once():
    dog = watchdog()
    dog.heartbeat(0.0, OPERATOR)
    dog.command(5.0)
    assert dog.tick(5.0)
    dog.heartbeat(5.5, OPERATOR)
    dog.command(5.5)
    assert not dog.tick(5.5) and not dog.reason(5.5) and not dog.release(5.5)


def test_node_holds_without_the_operator_and_survives_flips():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import flip_twice
    require_isolated_domain()
    from mhseals_nav.link_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[Parameter('max_age', value=0.5),
                                                Parameter('operator_hosts', value=[OPERATOR])])
        probe = rclpy.create_node('link_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        beat = probe.create_publisher(String, CFG['heartbeat_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def beats(host):
            def feed():
                beat.publish(String(data=host))
                cmd.publish(Twist())
            return feed

        # Bad: no operator yet (require_operator), then only the boat's own TUI beating.
        status = []
        probe.create_subscription(String, '/link_watchdog/status',
                                  lambda m: status.append(json.loads(m.data)), 10)
        flip_twice(ex, got, beats(host_name()), beats(OPERATOR))
        assert status and set(status[-1]) == {'ok', 'reason'}   # its liveness beat for the TUI
        assert dog.logic.beat_at is not None
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'operator_link'}
        assert {g['by'] for g in got} == {'link_watchdog'} and all(g['token'] for g in got)
    finally:
        rclpy.try_shutdown()


def test_never_counts_this_machine_named_by_astro_hostname(monkeypatch):
    """A container's name is not the machine's: its own beat names ASTRO_HOSTNAME."""
    monkeypatch.setenv('ASTRO_HOSTNAME', 'Laptop-A.local')
    dog = LinkWatchdog(dict(CFG, operator_hosts=['laptop-a', OPERATOR]))
    assert dog.own == host_name() == 'laptop-a'
    assert not dog.heartbeat(0.0, 'laptop-a') and dog.heartbeat(0.0, OPERATOR)
