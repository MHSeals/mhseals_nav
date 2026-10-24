"""Execute actual main closures with inert node, clock, service and worker."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from mhseals_nav import mavros_setup as setup
from mhseals_nav.mavros_configuration import resolve_setup, validate_setup

ROOT = Path(__file__).parents[1]


def inert_main(monkeypatch, overrides=None):
    params = resolve_setup(ROOT / 'config/mavros_setup.yaml')
    params.update(overrides or {})
    seen = NS(params={}, callbacks={}, waits=[], retries=[], timers=[], logs=[])

    class Event:
        def set(self): pass
        def clear(self): pass
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
        def create_publisher(self, *args): return NS(publish=lambda msg: None)
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
        ROOT / 'config/mavros_setup.yaml'), rclpy=NS(init=lambda **kw: None,
        ok=lambda: next(active), try_shutdown=lambda: None),
        threading=NS(Event=Event, Thread=thread), time=NS(monotonic=lambda: seen.now),
        MessageInterval=NS(Request=lambda **kw: NS(**kw)), DiagnosticStatus=NS,
        KeyValue=NS, Mavlink=NS, RCIn=NS, State=NS, qos_profile_sensor_data=None,
        spin_until_shutdown=lambda node: None)
    exec(compile(ast.Module(body=[main], type_ignores=[]), '<actual main>', 'exec'), ns)
    seen.now = 20.
    ns['main']()
    return seen


def test_actual_worker_uses_captured_request_policy(monkeypatch):
    seen = inert_main(monkeypatch, dict(retry_attempts=2, retry_delay=.25,
                                     response_timeout=.75, service_wait=1.5,
                                     reboot_dedupe_window=2.))
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


@pytest.mark.parametrize('key,value', [('retry_attempts', True), ('retry_attempts', 1.5),
    ('retry_delay', -1), ('response_timeout', 0), ('service_wait', float('nan')),
    ('reboot_dedupe_window', float('inf'))])
def test_policy_invalid_before_resources(monkeypatch, key, value):
    with pytest.raises(ValueError):
        inert_main(monkeypatch, {key: value})


def test_policy_validation_has_no_defaults():
    with pytest.raises(ValueError):
        validate_setup({'message_rates': ['1:2']})
