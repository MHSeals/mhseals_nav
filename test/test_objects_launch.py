"""Check source selection and message remapping without starting ROS nodes."""

import ast
from pathlib import Path

import pytest


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
