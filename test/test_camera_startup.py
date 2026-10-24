"""Execute actual camera startup before any inert wrapper, relay or DDS side effect."""
import ast
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from mhseals_nav.camera_configuration import resolve_camera
import pytest

ROOT = Path(__file__).parents[1]


def factory(resolver, calls):
    source = ROOT / 'launch/camera.launch.py'
    functions = [node for node in ast.parse(source.read_text()).body
                 if isinstance(node, ast.FunctionDef)]

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    class Joined:
        def __init__(self, parts):
            self.parts = parts

        def perform(self, context):
            return '/'.join(map(str, self.parts))

    def collect(*args, **kwargs):
        calls.append((args, kwargs))
        return kwargs
    namespace = {'LaunchConfiguration': Configuration, 'from_launch': resolver,
                 'Node': collect, 'IncludeLaunchDescription': collect, 'GroupAction': collect,
                 'SetEnvironmentVariable': collect, 'PythonLaunchDescriptionSource': lambda x: x,
                 'PathJoinSubstitution': Joined, 'FindPackageShare': lambda name: str(ROOT),
                 'os': os, 'tempfile': tempfile, 'ET': ET,
                 'FASTDDS_NS': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


@pytest.mark.parametrize('stage,hz', [
    ('package', 3), ('deployment', 4), ('operator', 5), ('alias', 6)])
def test_each_layer_reaches_actual_throttle(tmp_path, monkeypatch, stage, hz):
    """No Python rate default can hide edits to even the private package authority."""
    monkeypatch.delenv('ASTRO_DDS_ADDRESS', raising=False)
    paths = [tmp_path / name for name in ('package', 'deployment', 'operator')]
    for path, value in zip(paths, (3, 4, 5)):
        path.write_text(f'preview_hz: {value}\n')
    count = ['package', 'deployment', 'operator', 'alias'].index(stage) + 1
    chosen = [str(path) if i < count else '' for i, path in enumerate(paths)]
    result = resolve_camera(*chosen, preview_hz='6' if stage == 'alias' else '')
    calls = []
    factory(lambda context: result, calls)['camera_startup'](
        {'camera_name': 'front', 'camera_model': 'zed2i', 'slam': 'false'})
    throttle = next(kwargs for _, kwargs in calls if kwargs.get('executable') == 'throttle')
    assert throttle['arguments'][2] == str(float(hz))
    assert throttle['parameters'] == [{'lazy': True}]
    assert len([kwargs for _, kwargs in calls if kwargs.get('package') == 'topic_tools']) == 2
    include = next(kwargs for _, kwargs in calls if 'launch_arguments' in kwargs)
    assert dict(include['launch_arguments'])['publish_tf'] == 'false'


@pytest.mark.parametrize('raw', [
    'preview_hz: false', 'preview_hz: .nan',
    'preview_hz: 2\npreview_hz: 3', 'unknown: 2', 'missing'])
def test_invalid_masked_layer_never_constructs_or_creates_dds(tmp_path, monkeypatch, raw):
    """Reject input before actual pinned_profile can write a temporary DDS profile."""
    monkeypatch.setenv('ASTRO_DDS_ADDRESS', '192.168.8.20')
    path = tmp_path / 'bad.yaml'
    if raw != 'missing':
        path.write_text(raw)
    temporary_profiles = []
    original = tempfile.mkstemp

    def temporary(*args, **kwargs):
        temporary_profiles.append(True)
        return original(*args, **kwargs, dir=tmp_path)
    monkeypatch.setattr(tempfile, 'mkstemp', temporary)
    calls = []
    namespace = factory(lambda context: resolve_camera(
        ROOT / 'config/camera_startup.yaml', path, preview_hz='9'), calls)
    with pytest.raises((ValueError, OSError)):
        namespace['camera_startup']({'camera_name': 'front', 'slam': 'false'})
    assert calls == [] and temporary_profiles == []
    assert list(tmp_path.glob('camera_dds_*')) == []


@pytest.mark.parametrize('name', ['front', 'rear'])
@pytest.mark.parametrize('sim', ['true', 'false'])
def test_relay_destinations_match_actual_rgbd_consumer_remaps(name, sim):
    """Check wiring only; this does not activate dormant RGB-D odometry."""
    calls = []
    namespace = factory(lambda context: None, calls)
    produced = {node['name']: node['arguments'][-1]
                for node in namespace['relays'](name, True, 2.)}
    for filename, function in [('odom.launch.py', 'create_rtabmap_odom_node'),
                               ('slam.launch.py', 'create_rtabmap_slam_node')]:
        source = ROOT / 'launch' / filename
        node = next(item for item in ast.parse(source.read_text()).body
                    if isinstance(item, ast.FunctionDef) and item.name == function)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), namespace)
        consumer = namespace[function]({'sim': sim, 'camera_name': name,
                                       'rtabmap_params_file': '/selected/rtabmap.yaml'})[0]
        targets = dict(consumer['remappings'])
        assert targets['/rgb/camera_info'] == produced[f'{name}_info_relay']
        assert targets['/rgb/image'] == produced[f'{name}_rgb_relay']
        assert targets['/depth/image'] == produced[f'{name}_depth_relay']
        assert consumer['parameters'][1]['use_sim_time'] is (sim == 'true')
    assert len(namespace['relays'](name, False, 2.)) == 2
