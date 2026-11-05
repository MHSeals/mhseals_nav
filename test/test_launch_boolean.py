"""Exercise actual launch factory functions without requiring a ROS daemon."""
import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('sim', ['false', 'true'])
def test_sim_clock_is_boolean(sim):
    source = Path(__file__).parents[1] / 'launch' / 'slam.launch.py'
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


@pytest.mark.parametrize('text, value', [('1', True), ('True', True), ('0', False),
                                         ('false', False)])
def test_one_strict_parser(text, value):
    from mhseals_nav.layers import boolean
    assert boolean(text) is value


@pytest.mark.parametrize('text', ['yes', '', 'on', None])
def test_one_strict_parser_rejects(text):
    from mhseals_nav.layers import boolean
    with pytest.raises(ValueError, match='invalid launch boolean'):
        boolean(text)


def test_no_other_launch_boolean_parser():
    root = Path(__file__).parents[1]
    for path in [*root.glob('launch/*.py'), *root.glob('mhseals_nav/*.py')]:
        text = path.read_text()
        assert "== 'true'" not in text and 'def launch_boolean' not in text, path
    assert 'def boolean' not in (root / 'mhseals_nav/startup_projection.py').read_text()


def test_velodyne_driver_and_transform_use_same_model(tmp_path):
    from mhseals_nav.lidar_configuration import resolve_lidar
    from test_lidar_configuration import share as calibration_share
    from test_lidar_startup import context, sensor_factory

    share = calibration_share.__wrapped__(tmp_path)
    # authoritative package YAML + validated calibration + LIDAR_IP from the test env file
    resolved = resolve_lidar(share=share)
    nodes = sensor_factory(share)['sensor_nodes'](context())
    driver, transform = [node['parameters'][0] for node in nodes[:2]]
    assert driver == resolved['driver']
    assert transform == resolved['transform']
    assert driver['model'] == transform['model'] == 'VLP16'
    assert driver['frame_id'] == transform['frame_id'] == 'lidar_link'

