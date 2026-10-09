"""Exercise actual launch factory functions without requiring a ROS daemon."""
import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('filename,function', [
    ('odom.launch.py', 'create_rtabmap_odom_node'),
    ('slam.launch.py', 'create_rtabmap_slam_node'),
])
@pytest.mark.parametrize('sim', ['false', 'true'])
def test_sim_clock_is_boolean(filename, function, sim):
    source = Path(__file__).parents[1] / 'launch' / filename
    tree = ast.parse(source.read_text())
    # Select the factory containing RTABMap parameters, independent of name.
    factory = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                   and 'rtabmap_params_file' in ast.unparse(node)
                   and node.name.startswith('create_'))

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return sim if self.name == 'sim' else 'placeholder'

    namespace = {'LaunchConfiguration': Configuration, 'Node': lambda **kw: kw}
    exec(compile(ast.Module(body=[factory], type_ignores=[]), str(source), 'exec'), namespace)
    nodes = namespace[factory.name](None)
    assert nodes[0]['parameters'][1]['use_sim_time'] is (sim == 'true')


def test_velodyne_driver_and_transform_use_same_model():
    source = (Path(__file__).parents[1] / 'launch' /
              'sensors.launch.py').read_text()
    assert source.count("'model': 'VLP16'") == 2
