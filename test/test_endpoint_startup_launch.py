"""Actual endpoint factories and robot forwarding with inert constructor collectors."""
import ast
from pathlib import Path

from mhseals_nav import endpoint_configuration as endpoint
from mhseals_nav.odometry_configuration import resolve_odometry
import pytest
from test_odometry_startup_launch import factories
from test_robot_overlay import robot_factory
from test_sensor_startup import factory as sensor_factory
import yaml

ROOT = Path(__file__).parents[1]


def bind(namespace, calls, selection='', startup_context='standalone', aliases=None):
    """Bind actual pure resolver to an explicit package authority for inert execution."""
    def resolve(context):
        return endpoint.resolve_endpoints(
            ROOT / 'config/endpoint_startup.yaml', operator_file=selection,
            aliases=aliases, context=startup_context)
    namespace['endpoint_from_launch'] = resolve
    namespace['endpoint_boolean'] = endpoint.boolean
    namespace['Node'] = lambda **kwargs: calls.append(kwargs) or kwargs
    return namespace


@pytest.mark.parametrize('data,sim,context,expected', [
    ({'fcu_url': 'tcp://explicit:9000'}, False, 'standalone', 'tcp://explicit:9000'),
    ({'simulation_fcu_url': 'tcp://simulation:9001'}, True, 'standalone',
     'tcp://simulation:9001'),
    ({'serial_fallback': '/dev/selected'}, False, 'standalone',
     'serial:///dev/selected:115200'),
    ({'contexts': {'standalone': {'baud': 70000}}}, False, 'standalone',
     'serial:///dev/ttyACM0:70000'),
    ({'contexts': {'full_robot': {'baud': 70001}}}, False, 'full_robot',
     'serial:///dev/ttyACM0:70001'),
    ({'contexts': {'tui': {'baud': 70002}}}, False, 'tui', 'serial:///dev/ttyACM0:70002'),
])
def test_each_fcu_yaml_field_reaches_actual_node(
        tmp_path, monkeypatch, data, sim, context, expected):
    """Endpoint selection owns concrete FCU values without opening the chosen transport."""
    selected = tmp_path / 'selected.yaml'
    selected.write_text(yaml.safe_dump(data))
    monkeypatch.setattr(endpoint, 'discover_device', lambda *args: None)
    calls = []
    namespace = bind(factories('odom.launch.py', lambda: None, calls), calls, selected, context)
    node = namespace['create_mavros_node']({'sim': str(sim).lower()})[0]
    assert node['parameters'][1]['fcu_url'] == expected
    assert node['parameters'][1]['use_sim_time'] is sim


@pytest.mark.parametrize('context', endpoint.CONTEXTS)
def test_context_gcs_yaml_reaches_actual_node(tmp_path, context):
    """Each context's GCS URL supports explicit disable independently of inherit."""
    selected = tmp_path / 'selected.yaml'
    selected.write_text(yaml.safe_dump({'contexts': {context: {'gcs_url': ''}}}))
    calls = []
    namespace = bind(factories('odom.launch.py', lambda: None, calls), calls, selected, context)
    assert namespace['create_mavros_node']({'sim': 'true'})[0]['parameters'][1]['gcs_url'] == ''


@pytest.mark.parametrize('context', endpoint.CONTEXTS)
def test_context_discovery_yaml_changes_actual_node(tmp_path, context):
    """Fixed named strategy selection remains executable Python behind each context."""
    selected = tmp_path / 'selected.yaml'
    selected.write_text(yaml.safe_dump({'contexts': {context: {'discovery': 'tui_stable_hints'}}}))
    root = tmp_path / 'devices'
    root.mkdir()
    (root / 'ttyUSB0').touch()
    calls = []
    namespace = bind(factories('odom.launch.py', lambda: None, calls), calls, selected, context)
    resolved = namespace['endpoint_from_launch']({})

    class Config:
        def mavros_parameters(self, sim):
            return resolved.mavros_parameters(sim, root)
    namespace['endpoint_from_launch'] = lambda context: Config()
    assert str(root / 'ttyUSB0') in namespace['create_mavros_node'](
        {'sim': 'false'})[0]['parameters'][1]['fcu_url']


@pytest.mark.parametrize('data,expected', [
    ({'unity': {'ip': '0.0.0.0'}}, {'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 6767}),
    ({'unity': {'port': 7999}}, {'ROS_IP': '127.0.0.1', 'ROS_TCP_PORT': 7999}),
])
def test_unity_yaml_reaches_actual_typed_node(tmp_path, data, expected):
    """Unity receives an IPv4 string and integer port from validated YAML."""
    selected = tmp_path / 'selected.yaml'
    selected.write_text(yaml.safe_dump(data))
    calls = []
    namespace = bind(sensor_factory(), calls, selected)
    node = namespace['sensor_nodes']({'sim': 'true', 'sensors_ignore': ''})[0]
    assert node['parameters'][0] == expected
    assert type(node['parameters'][0]['ROS_TCP_PORT']) is int


@pytest.mark.parametrize('filename,function', [
    ('odom.launch.py', 'odometry_nodes'), ('odom.launch.py', 'create_mavros_node'),
    ('sensors.launch.py', 'sensor_nodes'), ('robot.launch.py', 'endpoint_preflight')])
def test_invalid_late_endpoint_fails_before_constructor(tmp_path, filename, function):
    """Even an unrelated late Unity field blocks the relevant endpoint closure."""
    selected = tmp_path / 'selected.yaml'
    selected.write_text('unity: {port: false}\n')
    calls = []
    namespace = bind(factories(filename, lambda: resolve_odometry(
        ROOT / 'config/odometry_startup.yaml'), calls), calls, selected)
    with pytest.raises(ValueError):
        namespace[function]({'sim': 'true', 'fcu': 'true'})
    assert calls == []


@pytest.mark.parametrize('clock', ['yes', '', 'none'])
def test_invalid_clock_precedes_node(clock):
    """MAVROS factory rejects nonboolean clocks before constructing its node."""
    calls = []
    namespace = bind(factories('odom.launch.py', lambda: None, calls), calls)
    with pytest.raises(ValueError):
        namespace['create_mavros_node']({'sim': clock})
    assert calls == []


def test_robot_forwards_layers_context_aliases_and_preflights_first():
    """Actual full robot includes carry one endpoint selection to odom and sensors."""
    entities = robot_factory()
    defaults = {item.args[0]: item.default_value for item in entities
                if hasattr(item, 'default_value')}
    assert defaults['endpoint_context'] == 'full_robot'
    assert defaults['fcu_url'] == defaults['ros_ip'] == defaults['ros_port'] == ''
    assert defaults['gcs_url'] == endpoint.UNSET_GCS
    selected = dict(defaults, endpoint_startup_file='/operator.yaml',
                    endpoint_deployment_file='/deployment.yaml')
    includes = [item for item in entities if hasattr(item, 'launch_arguments')]
    timer = next(item for item in entities if getattr(item, 'period', None) == 2.)
    includes.append(timer.actions[0])
    for include in includes:
        forwarded = dict(include.launch_arguments)
        if 'fcu_url' not in forwarded:
            continue
        for name in (*endpoint.LAYER_ARGUMENTS, 'endpoint_context', *endpoint.ALIASES):
            assert forwarded[name].perform(selected) == selected[name]
    # The first executable action is the gate, before includes and their child factories.
    first = next(item for item in entities if not hasattr(item, 'default_value'))
    assert first.function(None) == []  # inert fixture binds gate; real gate tested above
    source = ast.parse((ROOT / 'launch/robot.launch.py').read_text())
    gate = next(node for node in ast.walk(source) if isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'OpaqueFunction')
    assert gate.keywords[0].value.id == 'endpoint_preflight'
