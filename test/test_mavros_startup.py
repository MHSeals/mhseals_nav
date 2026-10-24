"""Construct actual launch factories with inert launch objects only."""
import ast
from pathlib import Path
from types import SimpleNamespace

from mhseals_nav.mavros_configuration import ALIASES, resolve_setup
import pytest
from test_robot_overlay import robot_factory
from test_mavros_configuration import selected

ROOT = Path(__file__).parents[1]


def factory():
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
              FindPackageShare=lambda x: str(ROOT), ALIASES=ALIASES,
              resolve_setup=resolve_setup)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), ns)
    return ns


def defaults(entities):
    return {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}


@pytest.mark.parametrize('explicit', ['', '20.0'])
def test_nondefault_robot_forwarded_and_direct_equivalent(tmp_path, explicit):
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
    ns = factory()
    context = defaults(ns['generate_launch_description']())
    context.update(mavros_setup_params_file=selected(tmp_path, ['31:7']),
                   mavros_imu_rate='nan')
    ns['Node'] = lambda **kwargs: pytest.fail('constructed node before validation')
    with pytest.raises(ValueError):
        ns['create_mavros_setup_node'](context)


def test_fcu_gate_still_contains_setup_and_sim_default():
    ns = factory()
    entities = ns['generate_launch_description']()
    group = next(e for e in entities if hasattr(e, 'actions'))
    assert group.condition.args[0].name == 'fcu'
    assert group.actions[-1].function is ns['create_mavros_setup_node']
    assert defaults(entities)['sim'] == 'true'
