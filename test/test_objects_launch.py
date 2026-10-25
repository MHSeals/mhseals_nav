"""Check backend selection and the perception nodes without starting ROS nodes."""

import ast
from pathlib import Path

import pytest
import yaml


def object_nodes(sim, backend):
    source = Path(__file__).parents[1] / 'launch/objects.launch.py'
    tree = ast.parse(source.read_text())
    body = [node for node in tree.body
            if (isinstance(node, ast.FunctionDef) and node.name == 'nodes')
            or (isinstance(node, ast.Assign) and node.targets[0].id == 'BACKENDS')]
    values = dict(sim=sim, backend=backend)

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return values[self.name]

    namespace = dict(LaunchConfiguration=Configuration, Node=lambda **kwargs: kwargs,
                     PathJoinSubstitution=lambda parts: '/'.join(map(str, parts)),
                     FindPackageShare=lambda package: package)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['nodes'](None)


@pytest.mark.parametrize('sim,backend,executable', [
    ('false', 'zed', 'zed_backend'), ('true', 'sim', 'sim_backend')])
def test_backend_tracker_and_perception_costmap(sim, backend, executable):
    nodes = object_nodes(sim, backend)
    assert [n['executable'] for n in nodes] == [
        executable, 'object_tracker', 'nav2_costmap_2d', 'lifecycle_manager']
    clock = {'use_sim_time': sim == 'true'}
    assert nodes[0]['parameters'] == [f'mhseals_nav/config/{executable}.yaml', clock]
    assert nodes[1]['parameters'] == ['mhseals_nav/config/tracker.yaml', clock]
    costmap, manager = nodes[2:]
    assert (costmap['namespace'], costmap['name']) == ('perception', 'costmap')
    assert costmap['parameters'] == ['mhseals_nav/config/perception_costmap.yaml', clock]
    assert (manager['namespace'], manager['name']) == ('perception',
                                                       'lifecycle_manager_perception')
    assert manager['parameters'] == [{'autostart': True, 'node_names': ['costmap'],
                                      'bond_timeout': 0.0, **clock}]


@pytest.mark.parametrize('sim,backend', [('invalid', 'zed'), ('false', 'rosbridge')])
def test_invalid_arguments_fail_before_nodes_start(sim, backend):
    with pytest.raises(ValueError):
        object_nodes(sim, backend)


def test_zed_predicts_through_half_second_occlusions():
    config = Path(__file__).parents[1] / 'config'
    boat = yaml.safe_load((config / 'zed_boat.yaml').read_text())
    assert boat['/**']['ros__parameters']['object_detection']['prediction_timeout'] == .5

    loaded = yaml.safe_load((config / 'custom_object_detection.yaml').read_text())
    custom = loaded.get('/**', {}).get('ros__parameters', loaded)['object_detection']
    classes = {key: value for key, value in custom.items() if key.startswith('class_')}
    assert len(classes) == custom['custom_class_count']
    assert all(value['prediction_timeout_s'] == .5 for value in classes.values())


def test_custom_detection_file_is_a_ros_parameter_file():
    """The wrapper appends this file to the node's parameter files; rcl rejects any other shape."""
    path = Path(__file__).parents[1] / 'config/custom_object_detection.yaml'
    loaded = yaml.safe_load(path.read_text())
    assert list(loaded) == ['/**']
    assert list(loaded['/**']) == ['ros__parameters']
    assert loaded['/**']['ros__parameters']['object_detection']['od_enabled'] is True


def test_boat_wrapper_tracks_objects_on_gen1_without_area_memory():
    path = Path(__file__).parents[1] / 'config/zed_boat.yaml'
    loaded = yaml.safe_load(path.read_text())
    assert list(loaded) == ['/**'] and list(loaded['/**']) == ['ros__parameters']
    params = loaded['/**']['ros__parameters']
    tracking = params['pos_tracking']
    assert tracking['pos_tracking_enabled'] is True
    assert (tracking['pos_tracking_mode'], tracking['area_memory']) == ('GEN_1', False)
    detection = params['object_detection']
    assert detection['od_enabled'] is True
    assert detection['enable_tracking'] is True
    assert detection['prediction_timeout'] == .5
    assert params['general']['pub_resolution'] == 'CUSTOM'
    assert params['general']['pub_frame_rate'] <= params['general']['grab_frame_rate']


def test_image_plugin_parameter_carries_the_node_name():
    """image_transport strips only the namespace (/front), so the node name stays."""
    path = Path(__file__).parents[1] / 'config/zed_boat.yaml'
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    assert params['zed_node.rgb.color.rect.image.enable_pub_plugins'] == [
        'image_transport/compressed']
    assert 'rgb.color.rect.image.enable_pub_plugins' not in params
