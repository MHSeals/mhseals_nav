"""Execute real launch factories with inert constructor collectors."""
import ast
from pathlib import Path

from mhseals_nav.localization_configuration import compile_localization, simulation_clock
from mhseals_nav.odometry_configuration import ALIASES, LAYER_ARGUMENTS, resolve_odometry
import pytest

ROOT = Path(__file__).parents[1]


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def factories(filename, settings, calls):
    source = ROOT / 'launch' / filename
    tree = ast.parse(source.read_text())
    nodes = [x for x in tree.body if isinstance(x, ast.FunctionDef)]

    def collect(*args, **kwargs):
        calls.append(kwargs)
        return kwargs
    namespace = {
        'LaunchConfiguration': Configuration,
        'Node': collect,
        'TimerAction': collect,
        'IncludeLaunchDescription': collect,
        'GroupAction': collect,
        'OpaqueFunction': collect,
        'PythonLaunchDescriptionSource': lambda x: x,
        'PathJoinSubstitution': lambda x: x,
        'FindPackageShare': lambda x: x,
        'IfCondition': lambda x: x,
        'from_launch': lambda context: settings(),
        'compile_localization': compile_localization,
        'simulation_clock': simulation_clock,
        'ALIASES': ALIASES,
        'LAYER_ARGUMENTS': LAYER_ARGUMENTS,
    }
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


def context():
    return {**{name: '' for name in (*ALIASES, *LAYER_ARGUMENTS)},
            'sim': 'true', 'ekf_overlay_file': '', 'ekf_startup_context': 'included',
            'navsat_transform_config_file': str(ROOT / 'config/navsat_transform.yaml'),
            'ekf_local_config_file': str(ROOT / 'config/ekf_local.yaml'),
            'ekf_global_config_file': str(ROOT / 'config/ekf_global.yaml')}


@pytest.mark.parametrize('key,value,target', [
    ('mavros_velocity_source', 'fcu', 'velocity_source'),
    ('mavros_velocity_stddev', .8, 'velocity_stddev'),
    ('mavros_velocity_max_age', 2.3, 'gps_velocity_max_age'),
    ('mavros_velocity_min_stddev', .2, 'gps_velocity_min_stddev'),
    ('mavros_velocity_no_fix_stddev', 3., 'velocity_stddev_nofix'),
])
def test_actual_adapter_factory_uses_each_yaml_field(tmp_path, key, value, target):
    selected = tmp_path / 'operator.yaml'
    selected.write_text(f'{key}: {value}\n')

    def settings():
        return resolve_odometry(ROOT / 'config/odometry_startup.yaml', operator_file=selected)
    calls = []
    launch = factories('odom.launch.py', settings, calls)
    result = launch['odometry_nodes'](context())
    adapter = next(x for x in calls if x.get('name') == 'mavros_odom_adapter')
    assert adapter['parameters'][0][target] == value
    included = next(x for x in calls if 'launch_arguments' in x)
    forwarded = dict(included['launch_arguments'])
    assert {*ALIASES, *LAYER_ARGUMENTS, 'ekf_startup_context'} <= set(forwarded)
    assert result


@pytest.mark.parametrize('startup_context,delay', [('standalone', 1.2), ('included', 7.8)])
def test_actual_ekf_factory_uses_context_delay(tmp_path, startup_context, delay):
    selected = tmp_path / 'operator.yaml'
    selected.write_text('ekf_delay_standalone: 1.2\nekf_delay_included: 7.8\n')
    calls = []

    def settings():
        return resolve_odometry(ROOT / 'config/odometry_startup.yaml',
                                operator_file=selected, context=startup_context)
    result = factories('ekf.launch.py', settings, calls)['ekf_nodes'](context())
    assert result[1]['period'] == delay


def test_invalid_layers_fail_before_any_constructor(tmp_path):
    selected = tmp_path / 'operator.yaml'
    selected.write_text('mavros_velocity_stddev: false\n')
    calls = []

    def settings():
        return resolve_odometry(ROOT / 'config/odometry_startup.yaml', operator_file=selected)
    routes = [('odom.launch.py', 'odometry_nodes'), ('ekf.launch.py', 'ekf_nodes')]
    for filename, function in routes:
        with pytest.raises(ValueError):
            factories(filename, settings, calls)[function](context())
    assert calls == []


def test_robot_state_frequency(tmp_path):
    selected = tmp_path / 'operator.yaml'
    selected.write_text('robot_state_publish_frequency: 42.0\n')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot/>')
    calls = []

    def settings():
        return resolve_odometry(ROOT / 'config/odometry_startup.yaml', operator_file=selected)
    namespace = factories('odom.launch.py', settings, calls)
    import os
    namespace['os'] = os
    result = namespace['load_urdf']({**context(), 'urdf_file': str(urdf)})
    assert result[0]['parameters'][0]['publish_frequency'] == 42.


def test_full_robot_forwards_startup_and_localization_selections():
    calls = []
    namespace = factories('robot.launch.py', lambda: None, calls)
    namespace.update(LaunchDescription=lambda x: x,
                     DeclareLaunchArgument=lambda *args, **kwargs: None,
                     persistent_limits_path=lambda: '', snapshot_root=lambda: '',
                     snapshot_scope=lambda: '')
    namespace['generate_launch_description']()
    # Identify the odom include by its complete localization forwarding surface.
    candidates = [dict(x['launch_arguments']) for x in calls if 'launch_arguments' in x]
    selected = next(x for x in candidates if 'urdf_file' in x and 'fcu_url' in x)
    required = (*ALIASES, *LAYER_ARGUMENTS, 'ekf_startup_context', 'ekf_overlay_file',
                'navsat_transform_config_file', 'ekf_local_config_file', 'ekf_global_config_file')
    assert all(selected[name].name == name for name in required)


@pytest.mark.parametrize('alias', [False, True])
def test_integer_spelled_frequency_projects_as_double(tmp_path, alias):
    selected = tmp_path / 'operator.yaml'
    selected.write_text('robot_state_publish_frequency: 50\nmavros_velocity_stddev: 1\n')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot/>')

    def settings():
        return resolve_odometry(
            ROOT / 'config/odometry_startup.yaml', operator_file=selected,
            aliases={'robot_state_publish_frequency': 60,
                     'mavros_velocity_stddev': 2} if alias else {})
    calls = []
    namespace = factories('odom.launch.py', settings, calls)
    import os
    namespace['os'] = os
    robot = namespace['load_urdf']({**context(), 'urdf_file': str(urdf)})[0]
    assert type(robot['parameters'][0]['publish_frequency']) is float
    namespace['odometry_nodes'](context())
    adapter = next(x for x in calls if x.get('name') == 'mavros_odom_adapter')
    assert type(adapter['parameters'][0]['velocity_stddev']) is float
    assert adapter['parameters'][0]['velocity_stddev'] == (2. if alias else 1.)
