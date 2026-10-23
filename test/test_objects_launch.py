"""Check source selection and message remapping without starting ROS nodes."""

import ast
from pathlib import Path

import pytest
import yaml


def object_nodes(sim, url='', root=''):
    source = Path(__file__).parents[1] / 'launch/objects.launch.py'
    factory = next(node for node in ast.parse(source.read_text()).body
                   if isinstance(node, ast.FunctionDef) and node.name == 'nodes')
    values = dict(sim=sim, rosbridge_url=url, camera_root_frame=root,
                  detections_topic='/detections',
                  objects_topic='/front/zed_node/obj_det/objects')

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return values[self.name]

    namespace = dict(LaunchConfiguration=Configuration,
                     ParameterValue=lambda value, **_: value.perform(None) == 'true',
                     Node=lambda **kwargs: kwargs)
    exec(compile(ast.Module(body=[factory], type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace['nodes'](None)


def test_simulation_needs_only_the_canonical_tracker():
    nodes = object_nodes('true')
    assert [node['executable'] for node in nodes] == ['object_tracker']
    assert nodes[0]['parameters'][0]['use_sim_time'] is True


def test_zed_adapter_remaps_input_and_preserves_tracker_contract():
    tracker, adapter = object_nodes('false')
    assert tracker['executable'] == 'object_tracker'
    assert tracker['parameters'][0]['use_sim_time'] is False
    assert adapter['executable'] == 'zed_detections_converter'
    assert adapter['parameters'][0]['objects_topic'] == 'objects'
    remaps = dict(adapter['remappings'])
    assert remaps['objects'].perform(None) == '/front/zed_node/obj_det/objects'
    for node in (tracker, adapter):
        assert dict(node['remappings'])['detections'].perform(None) == '/detections'


def test_bridge_uses_remote_topic_name_rather_than_local_ros_remapping():
    _, adapter = object_nodes('false', 'ws://camera:9090', 'front_camera_link')
    params = adapter['parameters'][0]
    assert params['rosbridge_url'] == 'ws://camera:9090'
    assert params['objects_topic'].perform(None) == '/front/zed_node/obj_det/objects'
    assert 'objects' not in dict(adapter['remappings'])


@pytest.mark.parametrize('sim,url,root', [
    ('invalid', '', ''),
    ('true', 'ws://camera:9090', 'front_camera_link'),
    ('false', 'ws://camera:9090', ''),
])
def test_invalid_source_combinations_fail_before_nodes_start(sim, url, root):
    with pytest.raises(ValueError):
        object_nodes(sim, url, root)


def test_bridge_arguments_are_declared_with_empty_defaults():
    source = Path(__file__).parents[1] / 'launch/objects.launch.py'
    text = source.read_text()
    for name in ('rosbridge_url', 'camera_root_frame'):
        assert f'DeclareLaunchArgument("{name}", default_value="")' in text


def test_jp6_wrapper_disables_sdk_predictions():
    config = Path(__file__).parents[1] / 'config'
    boat = yaml.safe_load((config / 'zed_boat.yaml').read_text())
    assert boat['/**']['ros__parameters']['object_detection']['prediction_timeout'] == 0

    loaded = yaml.safe_load((config / 'custom_object_detection.yaml').read_text())
    custom = loaded.get('/**', {}).get('ros__parameters', loaded)['object_detection']
    classes = {key: value for key, value in custom.items() if key.startswith('class_')}
    assert len(classes) == custom['custom_class_count']
    assert all(value['prediction_timeout_s'] == 0 for value in classes.values())


def test_custom_detection_file_is_a_ros_parameter_file():
    """The wrapper appends this file to the node's parameter files; rcl rejects any other shape."""
    path = Path(__file__).parents[1] / 'config/custom_object_detection.yaml'
    loaded = yaml.safe_load(path.read_text())
    assert list(loaded) == ['/**']
    assert list(loaded['/**']) == ['ros__parameters']
    assert loaded['/**']['ros__parameters']['object_detection']['od_enabled'] is True


def test_boat_wrapper_overrides_keep_zed_out_of_tf_and_odometry():
    path = Path(__file__).parents[1] / 'config/zed_boat.yaml'
    loaded = yaml.safe_load(path.read_text())
    assert list(loaded) == ['/**'] and list(loaded['/**']) == ['ros__parameters']
    params = loaded['/**']['ros__parameters']
    assert params['pos_tracking']['pos_tracking_enabled'] is False
    detection = params['object_detection']
    assert detection['od_enabled'] is True
    assert detection['enable_tracking'] is False
    assert detection['prediction_timeout'] == 0
    assert params['general']['pub_resolution'] == 'CUSTOM'
    assert params['general']['pub_frame_rate'] <= params['general']['grab_frame_rate']


def test_image_plugin_parameter_carries_the_node_name():
    """image_transport strips only the namespace (/front), so the node name stays."""
    path = Path(__file__).parents[1] / 'config/zed_boat.yaml'
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    assert params['zed_node.rgb.color.rect.image.enable_pub_plugins'] == [
        'image_transport/compressed']
    assert 'rgb.color.rect.image.enable_pub_plugins' not in params
