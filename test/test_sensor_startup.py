"""Execute the sensor factory with inert launch imports and no processes."""
import ast
from pathlib import Path

import pytest


def factory():
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
                 'large_data_env': {}}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


@pytest.mark.parametrize('sim', ['True', 'true', '1', 'TRUE'])
def test_simulation_never_constructs_physical_nodes(sim):
    nodes = factory()['sensor_nodes']({'sim': sim, 'sensors_ignore': ''})
    assert [n['package'] for n in nodes] == ['ros_tcp_endpoint']


@pytest.mark.parametrize('sim', ['false', '0', 'False'])
def test_physical_selection_and_configured_ip(sim):
    nodes = factory()['sensor_nodes']({'sim': sim, 'sensors_ignore': ''})
    assert [n['package'] for n in nodes] == [
        'velodyne_driver', 'velodyne_pointcloud', 'mhseals_nav']
    assert nodes[0]['parameters'][0]['device_ip'].name == 'velodyne_ip'
    assert factory()['sensor_nodes']({'sim': sim, 'sensors_ignore': ' lidar '}) == []


def test_invalid_boolean_fails_before_any_node():
    namespace = factory()
    namespace['Node'] = lambda **kw: pytest.fail('constructed a node')
    with pytest.raises(ValueError, match='Invalid sim boolean'):
        namespace['sensor_nodes']({'sim': 'yes', 'sensors_ignore': ''})
