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


def test_velodyne_driver_and_transform_use_same_model(tmp_path):
    from mhseals_nav.lidar_configuration import resolve_lidar
    from test_lidar_configuration import share as calibration_share
    from test_lidar_startup import context, sensor_factory

    share = calibration_share.__wrapped__(tmp_path)
    # authoritative package YAML + validated calibration + LIDAR_IP from the test env file
    resolved = resolve_lidar(share=share, fallback_ip='192.168.0.201')
    nodes = sensor_factory(share)['sensor_nodes'](context())
    driver, transform = [node['parameters'][0] for node in nodes[:2]]
    assert driver == resolved['driver']
    assert transform == resolved['transform']
    assert driver['model'] == transform['model'] == 'VLP16'
    assert driver['frame_id'] == transform['frame_id'] == 'lidar_link'


def test_imu_filter_is_opt_in():
    source = (Path(__file__).parents[1] / 'launch' / 'odom.launch.py').read_text()
    assert "('use_imu_filter', 'false'" in source
    assert "condition=IfCondition(launch_configurations['use_imu_filter'])" in source
