"""Execute the sensor factory with inert launch imports and no processes."""
import ast
from pathlib import Path

from mhseals_nav.endpoint_configuration import (
    LAYER_ARGUMENTS as ENDPOINT_LAYERS, resolve_endpoints, UNSET_GCS)
from mhseals_nav.lidar_configuration import resolve_lidar

import pytest

from test_lidar_configuration import share  # noqa: F401


def factory(lidar_resolver=resolve_lidar):
    path = Path(__file__).parents[1] / 'launch/sensors.launch.py'
    tree = ast.parse(path.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]
    namespace = {'LaunchConfiguration': Configuration, 'Node': lambda **kw: kw,
                 'PathJoinSubstitution': lambda x: x, 'FindPackageShare': lambda x: x,
                 'large_data_env': {}, 'resolve_lidar': lidar_resolver,
                 'endpoint_from_launch': lambda context: resolve_endpoints(
                     Path(__file__).parents[1] / 'config/endpoint_startup.yaml'),
                 'ENDPOINT_LAYERS': ENDPOINT_LAYERS, 'UNSET_GCS': UNSET_GCS,
                 'DeclareLaunchArgument': lambda name, **kw: dict(name=name, **kw),
                 'OpaqueFunction': lambda **kw: kw, 'LaunchDescription': list}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


@pytest.mark.parametrize('sim', ['True', 'true', '1', 'TRUE'])
def test_simulation_never_constructs_physical_nodes(sim):
    namespace = factory(lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    nodes = namespace['sensor_nodes']({
        'sim': sim, 'sensors_ignore': '', 'lidar_params_file': '',
        'lidar_deployment_file': '', 'lidar_required_frame': '',
        'velodyne_ip': '10.2.3.4', 'velodyne_port': ''})
    assert [n['package'] for n in nodes] == ['ros_tcp_endpoint']


@pytest.mark.parametrize('sim', ['false', '0', 'False'])
def test_physical_selection_and_configured_ip(sim, share):
    namespace = factory(lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    nodes = namespace['sensor_nodes']({
        'sim': sim, 'sensors_ignore': '', 'lidar_params_file': '',
        'lidar_deployment_file': '', 'lidar_required_frame': '',
        'velodyne_ip': '10.2.3.4', 'velodyne_port': ''})
    assert [n['package'] for n in nodes] == [
        'velodyne_driver', 'velodyne_pointcloud', 'mhseals_nav']
    assert nodes[0]['parameters'][0]['device_ip'] == '10.2.3.4'
    assert factory()['sensor_nodes']({'sim': sim, 'sensors_ignore': ' lidar '}) == []


def test_invalid_boolean_fails_before_any_node():
    namespace = factory()
    namespace['Node'] = lambda **kw: pytest.fail('constructed a node')
    with pytest.raises(ValueError, match='Invalid sim boolean'):
        namespace['sensor_nodes']({'sim': 'yes', 'sensors_ignore': ''})
