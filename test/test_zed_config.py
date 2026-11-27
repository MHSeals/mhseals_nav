"""The ZED wrapper's parameter files (config/zed_boat.yaml, custom_object_detection.yaml)."""

from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]


def flat(tree, prefix=''):
    """Dotted ROS parameter names of a nested YAML mapping."""
    names = {}
    for key, value in tree.items():
        if isinstance(value, dict):
            names.update(flat(value, f'{prefix}{key}.'))
        else:
            names[f'{prefix}{key}'] = value
    return names


def ros_params(path):
    return yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']


def wrapper_parameters(*names):
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('zed_wrapper'), 'config')
    except Exception:
        pytest.skip('zed_wrapper is not installed')
    found = {}
    for name in names:
        found.update(flat(ros_params(share / name)))
    return found


def test_boat_overrides_name_only_parameters_the_wrapper_reads():
    """A misspelt key would be declared by nobody and silently ignored."""
    known = wrapper_parameters('common_stereo.yaml', 'zed2i.yaml')
    ours = flat(ros_params(ROOT / 'config/zed_boat.yaml'))
    # image_transport reads its own per-topic parameter, not the wrapper.
    assert {k for k in ours if not k.startswith('zed_node.')} <= set(known)


def test_custom_detection_runs_epoch60_with_wrapper_class_keys():
    known = wrapper_parameters('custom_object_detection.yaml')
    sample = {k.split('.', 2)[2] for k in known if k.startswith('object_detection.class_000.')}
    detection = ros_params(ROOT / 'config/custom_object_detection.yaml')['object_detection']
    assert detection['detection_model'] == 'CUSTOM_YOLOLIKE_BOX_OBJECTS'
    assert 'custom_onnx_file' not in detection        # the launch passes the share's model
    assert (ROOT / 'config/epoch60.onnx').is_file()
    classes = {k: v for k, v in detection.items() if k.startswith('class_')}
    assert len(classes) == detection['custom_class_count']
    assert all(set(v) <= sample for v in classes.values())


@pytest.mark.parametrize('name', ['zed_boat.yaml', 'custom_object_detection.yaml'])
def test_rcl_accepts_the_parameter_file(name):
    """The rcl layer parses --params-file at init and rejects any non-ROS shape."""
    rclpy = pytest.importorskip('rclpy')
    context = rclpy.context.Context()
    rclpy.init(args=['--ros-args', '--params-file', str(ROOT / 'config' / name)],
               context=context)
    rclpy.shutdown(context=context)


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
