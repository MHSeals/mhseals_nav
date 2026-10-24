"""Construct actual launch factories with inert launch objects only."""
import ast
from pathlib import Path
from types import SimpleNamespace

from mhseals_nav.mavros_configuration import ALIASES, resolve_setup
from mhseals_nav.odometry_configuration import (
    ALIASES as ODOMETRY_ALIASES, LAYER_ARGUMENTS, resolve_odometry)

import pytest

from test_mavros_configuration import selected

from test_robot_overlay import robot_factory

ROOT = Path(__file__).parents[1]


def factory():
    """Bind actual odometry factories to inert launch constructors."""
    source = ROOT / 'launch/odom.launch.py'
    tree = ast.parse(source.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]

    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return context[self.name]

    def record(*args, **kwargs): return SimpleNamespace(args=args, **kwargs)
    ns = {name: record for name in (
        'Node', 'DeclareLaunchArgument', 'GroupAction', 'IncludeLaunchDescription',
        'OpaqueFunction', 'PythonLaunchDescriptionSource', 'IfCondition',
        'UnlessCondition', 'ParameterValue')}
    ns.update(LaunchConfiguration=Configuration, LaunchDescription=lambda x: x,
              PathJoinSubstitution=lambda x: '/'.join(map(str, x)),
              FindPackageShare=lambda x: str(ROOT), ALIASES=ODOMETRY_ALIASES,
              LAYER_ARGUMENTS=LAYER_ARGUMENTS,
              resolve_setup=resolve_setup,
              from_launch=lambda context: resolve_odometry(
                  ROOT / 'config/odometry_startup.yaml',
                  deployment_file=context['odometry_deployment_file'],
                  operator_file=context['odometry_startup_file'],
                  aliases={name: context[name] for name in ODOMETRY_ALIASES},
                  context=context['ekf_startup_context']))
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), ns)
    return ns


def defaults(entities):
    """Collect actual declared launch argument defaults."""
    return {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}


@pytest.mark.parametrize('explicit', ['', '20.0'])
def test_nondefault_robot_forwarded_and_direct_equivalent(tmp_path, explicit):
    """Nondefault robot forwarded and direct equivalent."""
    path = selected(tmp_path, ['33:7', '30:8', '31:9', '24:4', '27:3', '1:4', '193:6'])
    ns = factory()
    direct = defaults(ns['generate_launch_description']())
    assert all(direct[name] == '' for name in ALIASES)
    robot = robot_factory()
    robot_context = defaults(robot)
    assert all(robot_context[name] == '' for name in ALIASES)
    robot_context.update(mavros_setup_params_file=path, mavros_imu_rate=explicit)
    timer = next(e for e in robot if getattr(e, 'period', None) == 2.0)
    forwarded = {key: value.perform(robot_context)
                 for key, value in timer.actions[0].launch_arguments}
    direct.update(mavros_setup_params_file=path, mavros_imu_rate=explicit)
    a = ns['create_mavros_setup_node'](dict(direct, **forwarded))[0]
    b = ns['create_mavros_setup_node'](direct)[0]
    assert a.parameters == b.parameters
    assert a.parameters[0] == path
    rates = dict(entry.split(':') for entry in a.parameters[1]['message_rates'])
    assert {key: rates[key] for key in ('27', '1', '193')} == {'27': '3', '1': '4', '193': '6'}
    assert rates['30'] == ('20' if explicit else '8')
    assert rates['31'] == ('20' if explicit else '9')


@pytest.mark.parametrize('selection', ['', '/missing/rates.yaml', 'invalid'])
def test_selection_failure_precedes_node(tmp_path, selection):
    """Selection failure precedes node."""
    if selection == 'invalid':
        path = tmp_path / 'invalid.yaml'
        path.write_text('mavros_setup: []')
        selection = str(path)
    ns = factory()
    context = defaults(ns['generate_launch_description']())
    context['mavros_setup_params_file'] = selection
    ns['Node'] = lambda **kwargs: pytest.fail('constructed node before validation')
    with pytest.raises((ValueError, OSError)):
        ns['create_mavros_setup_node'](context)


def test_invalid_final_alias_precedes_node(tmp_path):
    """Invalid final alias precedes node."""
    ns = factory()
    context = defaults(ns['generate_launch_description']())
    context.update(mavros_setup_params_file=selected(tmp_path, ['31:7']),
                   mavros_imu_rate='nan')
    ns['Node'] = lambda **kwargs: pytest.fail('constructed node before validation')
    with pytest.raises(ValueError):
        ns['create_mavros_setup_node'](context)


def test_fcu_gate_still_contains_setup_and_sim_default():
    """Fcu gate still contains setup and sim default."""
    ns = factory()
    entities = ns['generate_launch_description']()
    opaque = next(e for e in entities if hasattr(e, 'function'))
    assert opaque.function is ns['odometry_nodes']
    group = next(e for e in opaque.function(defaults(entities)) if hasattr(e, 'actions'))
    assert group.condition.args[0].name == 'fcu'
    assert group.actions[-1].function is ns['create_mavros_setup_node']
    assert defaults(entities)['sim'] == 'true'


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
    assert ('fcu_sysid' in declarations) == (rates != ['31:nan'])


@pytest.mark.parametrize('alias,ids', list(ALIASES.items()))
def test_every_explicit_alias_overrides_and_unset_preserves(tmp_path, alias, ids):
    """Every explicit alias overrides and unset preserves."""
    ns = factory()
    context = defaults(ns['generate_launch_description']())
    context['mavros_setup_params_file'] = selected(tmp_path, ['33:7', '30:8', '31:9',
                                                              '24:4', '65:6'])
    original = ns['create_mavros_setup_node'](context)[0].parameters[1]
    context[alias] = '13.5'
    effective = ns['create_mavros_setup_node'](context)[0].parameters[1]
    before = dict(e.split(':') for e in original['message_rates'])
    after = dict(e.split(':') for e in effective['message_rates'])
    assert after == dict(before, **{str(i): '13.5' for i in ids})
