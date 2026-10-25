"""Execute actual main closures with inert node, clock, service and worker."""
import ast
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace as NS

from mhseals_nav import mavros_setup as setup
from mhseals_nav.mavros_configuration import resolve_setup, validate_rates, validate_setup
import pytest

ROOT = Path(__file__).parents[1]


def inert_main(monkeypatch, overrides=None):
    params = resolve_setup(ROOT / 'config/mavros_setup.yaml')
    params.update(overrides or {})
    seen = NS(params={}, callbacks={}, waits=[], retries=[], timers=[], logs=[],
              sets=[], published=[])

    class Event:
        def set(self): seen.sets.append(True)  # noqa: A003 (threading.Event API)

        def clear(self): pass

        def is_set(self): return False

        def wait(self, timeout=None):
            seen.waits.append(timeout)
            return True

    class Node:
        def __init__(self, name): pass

        def declare_parameter(self, name, value): seen.params[name] = params.get(name, value)

        def get_parameter(self, name): return NS(value=seen.params[name])

        def get_parameters_by_prefix(self, prefix):
            return {k: NS(value=v) for k, v in seen.params.items()}

        def add_on_set_parameters_callback(self, callback): seen.setter = callback

        def create_client(self, *args):
            def call_async(request):
                return NS(add_done_callback=lambda callback: callback(None),
                          result=lambda: NS(success=False))
            return NS(wait_for_service=lambda **kw: seen.waits.append(kw) or True,
                      call_async=call_async)

        def create_publisher(self, *args): return NS(publish=seen.published.append)

        def create_subscription(self, kind, topic, callback, qos): seen.callbacks[topic] = callback

        def create_timer(self, period, callback): seen.timers.append((period, callback))

        def get_logger(self): return NS(info=seen.logs.append, warning=seen.logs.append)

        def destroy_node(self): pass

    def thread(**kwargs):
        seen.worker = kwargs['target']
        return NS(start=lambda: None)

    tree = ast.parse((ROOT / 'mhseals_nav/mavros_setup.py').read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    main.body = [n for n in main.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    active = iter([True, False])
    ns = dict(vars(setup), Node=Node, installed_setup_defaults=lambda: resolve_setup(
        ROOT / 'config/mavros_setup.yaml'),
        rclpy=NS(init=lambda **kw: None, ok=lambda: next(active), try_shutdown=lambda: None),
        threading=NS(Event=Event, Thread=thread, Lock=threading.Lock),
        time=NS(monotonic=lambda: seen.now),
        MessageInterval=NS(Request=lambda **kw: NS(**kw)), DiagnosticStatus=NS,
        KeyValue=NS, Mavlink=NS, RCIn=NS, State=NS, qos_profile_sensor_data=None,
        spin_until_shutdown=lambda node: None)
    exec(compile(ast.Module(body=[main], type_ignores=[]), '<actual main>', 'exec'), ns)
    seen.now = 20.
    ns['main']()
    return seen


def test_actual_worker_uses_captured_request_policy(monkeypatch):
    seen = inert_main(monkeypatch, {'retry_attempts': 2, 'retry_delay': .25,
                                    'response_timeout': .75, 'service_wait': 1.5,
                                    'reboot_dedupe_window': 2.})
    monkeypatch.setattr(setup.time, 'sleep', lambda _: pytest.fail('real sleep'))
    # request_rates defaults to the real sleep: substitute only the sleep seam.
    original = setup.request_rates
    seen.worker.__globals__['request_rates'] = lambda *a, **kw: original(
        *a, **kw, sleep=seen.retries.append)
    seen.worker()
    assert seen.waits[1] == {'timeout_sec': 1.5}
    assert seen.waits.count(.75) == 16
    assert seen.retries == [.25] * 8
    cells = dict(zip(seen.worker.__code__.co_freevars, seen.worker.__closure__))
    with pytest.raises(TypeError):
        cells['policy'].cell_contents['retry_attempts'] = 99


@pytest.mark.parametrize('key,value', [
    ('retry_attempts', True), ('retry_attempts', 1.5), ('retry_delay', -1),
    ('response_timeout', 0), ('service_wait', float('nan')),
    ('reboot_dedupe_window', float('inf'))])
def test_policy_invalid_before_resources(monkeypatch, key, value):
    with pytest.raises(ValueError):
        inert_main(monkeypatch, {key: value})


def test_policy_validation_has_no_defaults():
    with pytest.raises(ValueError):
        validate_setup({'message_rates': ['1:2']})


def test_actual_rc_callbacks_use_diagnostic_policy(monkeypatch):
    seen = inert_main(monkeypatch, {'rc_window': 2., 'rc_min_hz': 1., 'rc_min_channels': 2,
                                    'rc_grace': 3., 'rc_timer': .2})
    assert seen.timers[0][0] == .2
    seen.callbacks['/mavros/state'](NS(connected=True))
    check = seen.timers[0][1]
    check()
    seen.now = 22.
    check()
    assert seen.logs == []
    seen.now = 23.
    for _ in range(2):
        seen.callbacks['/mavros/rc/in'](NS(channels=[1, 2]))
    check()
    assert seen.logs == []  # 2 frames / 2 seconds meets nondefault 1 Hz
    seen.now = 26.
    check()
    assert 'under 1 Hz or under 2 channels' in seen.logs[-1]
    seen.callbacks['/mavros/rc/in'](NS(channels=[1]))
    check()
    assert len(seen.logs) == 1


def test_actual_source_filter_and_dedupe_window(monkeypatch):
    seen = inert_main(monkeypatch, {'fcu_sysid': 7, 'reboot_dedupe_window': 2.})
    receive = seen.callbacks['/uas1/mavlink_source']

    def boot(sysid, boot_ms):
        raw = struct.pack('<QI', 0, boot_ms).ljust(16, b'\0')
        receive(NS(sysid=sysid, compid=1, msgid=2, len=12,
                   payload64=struct.unpack('<2Q', raw)))
    boot(7, 100)
    boot(1, 1)
    assert seen.sets == []
    boot(7, 90)
    assert len(seen.sets) == 1
    seen.now = 21.
    boot(7, 80)
    assert len(seen.sets) == 1
    seen.now = 22.5
    boot(7, 70)
    assert len(seen.sets) == 2


def test_startup_policy_live_changes_refused(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rcl_interfaces', NS())
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.msg', NS(SetParametersResult=NS))
    seen = inert_main(monkeypatch)
    for name in seen.params.keys() - {'message_rates'}:
        result = seen.setter([NS(name=name, value=999)])
        assert not result.successful and 'restart required' in result.reason


@pytest.mark.parametrize('key,value', [
    ('rc_window', 0), ('rc_min_hz', True), ('rc_min_channels', 1.5),
    ('rc_grace', -1), ('rc_timer', float('inf')), ('fcu_sysid', True),
    ('fcu_sysid', 0), ('fcu_sysid', 256)])
def test_monitor_identity_invalid_before_resources(monkeypatch, key, value):
    with pytest.raises(ValueError):
        inert_main(monkeypatch, {key: value})


def test_invalid_selected_policy_is_not_hidden_by_alias(tmp_path):
    path = tmp_path / 'selected.yaml'
    path.write_text((ROOT / 'config/mavros_setup.yaml').read_text().replace(
        'retry_attempts: 3', 'retry_attempts: true'))
    with pytest.raises(ValueError):
        resolve_setup(path, {'mavros_imu_rate': '19'})


def test_ekf_source_filter_applies_before_observation(monkeypatch):
    seen = inert_main(monkeypatch, {'fcu_sysid': 7})
    receive = seen.callbacks['/uas1/mavlink_source']
    receive(NS(sysid=1, compid=1, msgid=193))  # no payload access for other FCU
    assert seen.published == []
    receive(NS(sysid=7, compid=1, msgid=193, len=0, payload64=[]))
    assert len(seen.published) == 1


def rates_setter(monkeypatch):
    monkeypatch.setitem(sys.modules, 'rcl_interfaces', NS())
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.msg', NS(SetParametersResult=NS))
    return inert_main(monkeypatch)


@pytest.mark.parametrize('bad', [[], ['33:5', '33:6'], ['33:-5'], ['-1:5']])
def test_actual_setter_refuses_invalid_rates_and_requests_nothing(monkeypatch, bad):
    seen = rates_setter(monkeypatch)
    startup = validate_rates(list(seen.params['message_rates']))
    result = seen.setter([NS(name='message_rates', value=bad)])
    assert not result.successful and result.reason.startswith('message_rates ')
    assert seen.sets == []  # the worker is never woken, so no request goes out
    requested = []
    seen.worker.__globals__['request_rates'] = lambda rates, *a, **kw: requested.append(rates)
    seen.worker()
    assert requested == [startup]  # read-back: the accepted value is unchanged


def test_actual_setter_accepts_valid_rates_and_drops_nothing_implicitly(monkeypatch):
    seen = rates_setter(monkeypatch)
    result = seen.setter([NS(name='message_rates', value=['33:10', '31:0'])])
    assert result.successful and seen.sets == [True]
    requested = []
    seen.worker.__globals__['request_rates'] = lambda rates, *a, **kw: requested.append(rates)
    seen.worker()
    # Exactly the new list: ids the default set had but this one omits get no implicit 0.
    assert requested == [[(33, 10.), (31, 0.)]]


def fcu(msgid):
    """Return an autopilot-sourced raw MAVLink message of id ``msgid``."""
    return NS(sysid=1, compid=1, msgid=msgid, len=0, payload64=[])


def test_actual_verify_requests_again_only_the_short_ids(monkeypatch):
    """Request again only the ids that arrive under 0.7 of their rate."""
    # 2026-10-25: ATTITUDE (30) and ATTITUDE_QUATERNION (31) at 50 Hz failed at connect,
    # and ATTITUDE stayed at the FCU's SR0_EXTRA1 = 4 Hz; nothing asked again.
    seen = inert_main(monkeypatch, {'verify_period': 10.})
    period, verify = seen.timers[1]
    assert period == 10.
    seen.callbacks['/mavros/state'](NS(connected=True))
    seen.sets.clear()
    verify()                                  # starts the span
    wanted = dict(validate_rates(list(seen.params['message_rates'])))
    for msgid, hz in wanted.items():
        got = 4. if msgid == 30 else hz       # 30 at the stream default, 31 never sent
        for _ in range(int(got * 10)):
            if msgid != 31:
                seen.callbacks['/uas1/mavlink_source'](fcu(msgid))
    seen.now += 10.
    verify()
    assert seen.sets == [True]
    assert any('30 at 4.0 of 25 Hz' in line for line in seen.logs)
    requested = []
    seen.worker.__globals__['request_rates'] = lambda rates, *a, **kw: requested.append(rates)
    seen.worker()
    assert requested == [[(30, 25.), (31, 25.)]]


def test_actual_verify_waits_for_a_full_span_and_a_connection(monkeypatch):
    """Give no verdict before a full span while connected."""
    seen = inert_main(monkeypatch, {'verify_period': 10.})
    verify = seen.timers[1][1]
    verify()
    seen.now += 30.
    verify()                                  # not connected: never counts
    seen.callbacks['/mavros/state'](NS(connected=True))
    seen.sets.clear()
    verify()
    seen.now += 5.
    verify()                                  # half a span: no verdict yet
    assert seen.sets == []


def test_reconnect_after_a_partial_retry_requests_the_whole_list(monkeypatch):
    """Request the whole list on a reconnect, even after a partial retry."""
    seen = inert_main(monkeypatch, {'verify_period': 10.})
    seen.callbacks['/mavros/state'](NS(connected=True))
    verify = seen.timers[1][1]
    verify()
    seen.now += 10.
    verify()                                  # nothing arrived: every id is short
    seen.callbacks['/mavros/state'](NS(connected=False))
    seen.now += 10.
    seen.callbacks['/mavros/state'](NS(connected=True))
    requested = []
    seen.worker.__globals__['request_rates'] = lambda rates, *a, **kw: requested.append(rates)
    seen.worker()
    assert requested == [validate_rates(list(seen.params['message_rates']))]


def test_a_rates_change_after_a_partial_retry_requests_the_whole_new_list(monkeypatch):
    """A message_rates change landing before the worker reads a retry is not narrowed."""
    monkeypatch.setitem(sys.modules, 'rcl_interfaces', NS())
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.msg', NS(SetParametersResult=NS))
    seen = inert_main(monkeypatch, {'verify_period': 10.})
    seen.callbacks['/mavros/state'](NS(connected=True))
    verify = seen.timers[1][1]
    verify()
    seen.now += 10.
    for _ in range(250):                      # all but 30 and 31 arrive on time
        for msgid in (33, 24, 27, 1, 193, 65):
            seen.callbacks['/uas1/mavlink_source'](fcu(msgid))
    verify()                                  # wakes the worker for 30 and 31 only
    result = seen.setter([NS(name='message_rates', value=['33:10', '30:25', '65:5'])])
    assert result.successful
    requested = []
    seen.worker.__globals__['request_rates'] = lambda rates, *a, **kw: requested.append(rates)
    seen.worker()
    assert requested == [[(33, 10.), (30, 25.), (65, 5.)]]
