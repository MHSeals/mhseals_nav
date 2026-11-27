"""mavros_setup's main: its declarations and the rate check before the worker starts."""
import ast
from pathlib import Path
from types import SimpleNamespace

from mhseals_nav.mavros_configuration import resolve_setup

import pytest

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize('rates', [['31:17'], ['31:nan']])
def test_actual_main_declaration_and_pre_worker_validation(rates):
    """Actual main declaration and pre worker validation."""
    from mhseals_nav import mavros_setup
    from mhseals_nav.mavros_configuration import validate_rates

    tree = ast.parse((ROOT / 'mhseals_nav/mavros_setup.py').read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    # Bind imports to inert objects, retaining the actual executable body.
    main.body = [n for n in main.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    declarations = {}

    class ReachedLegacyBoundary(Exception):
        pass

    class InertNode:
        def __init__(self, name): assert name == 'mavros_setup'
        def declare_parameter(self, name, value): declarations[name] = value
        def get_parameter(self, name): return SimpleNamespace(value=declarations[name])

    def legacy_boundary(*args, **kwargs):
        raise ReachedLegacyBoundary

    ns = dict(vars(mavros_setup), Node=InertNode,
              rclpy=SimpleNamespace(init=lambda **kwargs: None),
              installed_setup_defaults=lambda: dict(
                  resolve_setup(ROOT / 'config/mavros_setup.yaml'), message_rates=rates),
              validate_rates=validate_rates, LiveParams=legacy_boundary)
    exec(compile(ast.Module(body=[main], type_ignores=[]), '<actual main>', 'exec'), ns)
    with pytest.raises(ValueError if rates == ['31:nan'] else ReachedLegacyBoundary):
        ns['main']()
    assert declarations['message_rates'] == rates
    assert declarations['fcu_sysid'] == 1
