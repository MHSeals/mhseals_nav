"""perception.launch.py: the camera, the lidar and the object pipeline, in the launch harness."""
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

from launch_harness import executables, harness, included, named, packages  # noqa: F401
from mhseals_nav.camera_configuration import resolve_camera
from mhseals_nav.layers import config_path
from mhseals_nav.lidar_configuration import resolve_lidar
import pytest
import test_lidar_configuration as calibration
from test_lidar_configuration import overlay
import yaml

ROOT = Path(__file__).parents[1]
CONFIG = str(config_path())   # layers' lookup: the source tree here, the share when installed
OBJECTS = ['object_tracker', 'nav2_costmap_2d', 'lifecycle_manager', 'nearest_obstacle']
CAMERA = {'zed': ['relay', 'throttle'], 'sim': [], 'none': []}   # the include is not a Node
BACKEND = {'zed': ['zed_backend'], 'sim': ['sim_backend'], 'none': []}
LIDAR = {'velodyne': ['velodyne_driver_node', 'velodyne_transform_node', 'lidar_relay', 'nearby'],
         'sim': ['nearby'], 'none': []}


@pytest.fixture
def share(tmp_path):
    """Give the calibrated installed-share fixture (velodyne_pointcloud's VLP16 table)."""
    return calibration.share.__wrapped__(tmp_path)


@pytest.fixture
def perception(harness, share, monkeypatch):  # noqa: F811
    """Load the module with its lidar resolved against the test share and no DDS address."""
    monkeypatch.delenv('ASTRO_DDS_ADDRESS', raising=False)
    module = harness.load('perception.launch.py')
    monkeypatch.setattr(module, 'resolve_lidar',
                        lambda *args, **kwargs: resolve_lidar(*args, share=share, **kwargs))
    return module


@pytest.fixture
def run(harness, perception):  # noqa: F811
    def start(**arguments):
        return harness.run('perception.launch.py', **arguments)
    start.harness, start.module = harness, perception
    return start


def fusion(module, monkeypatch, **values):
    """Overlay config/lidar_fusion.yaml with ``values`` for this launch."""
    read = module.params_file
    monkeypatch.setattr(module, 'params_file', lambda name: {
        **read(name), **(values if name == 'lidar_fusion.yaml' else {})})


@pytest.mark.parametrize('camera', ['zed', 'sim', 'none'])
@pytest.mark.parametrize('lidar', ['velodyne', 'sim', 'none'])
def test_camera_and_lidar_pick_the_drivers_and_the_backend(run, camera, lidar):
    records = run(camera=camera, lidar=lidar)
    assert executables(records) == CAMERA[camera] + LIDAR[lidar] + BACKEND[camera] + OBJECTS
    assert ('zed_wrapper' in packages(records)) is (camera == 'zed')
    assert [r for r in records if r.kind != 'Node'] == (
        [r for r in records if r.kind == 'IncludeLaunchDescription'] if camera == 'zed' else [])


@pytest.mark.parametrize('sim', ['True', 'true', '1', 'TRUE'])
def test_the_simulator_never_constructs_a_physical_driver(run, sim):
    run.module.resolve_lidar = lambda *args, **kwargs: pytest.fail('resolved the physical lidar')
    run.module.resolve_camera = lambda *args, **kwargs: pytest.fail('resolved the ZED')
    records = run(camera='sim', lidar='sim', sim=sim)
    assert executables(records) == ['nearby', 'sim_backend', *OBJECTS]
    assert not {'velodyne_driver', 'velodyne_pointcloud', 'zed_wrapper'} & packages(records)
    assert named(records, 'nearby')['parameters'] == [{'use_sim_time': True}]
    assert run(camera='none', lidar='none', sim=sim)   # the tracker and costmap still run


@pytest.mark.parametrize('arguments', [
    {'camera': 'rosbridge'}, {'camera': 'ZED'}, {'lidar': 'vlp16'}, {'sim': 'yes'},
    {'mapping': 'maybe'}])
def test_an_invalid_choice_fails_before_any_node(run, arguments):
    with pytest.raises(ValueError):
        run(**arguments)
    assert run.harness.built == []


# Objects

@pytest.mark.parametrize('camera,sim', [('zed', 'false'), ('sim', 'true'), ('none', 'false')])
def test_the_backend_tracker_and_perception_costmap(run, camera, sim):
    nodes = [r for r in run(camera=camera, lidar='none', sim=sim) if r.kind == 'Node'
             and r['executable'] not in CAMERA['zed']]
    clock = {'use_sim_time': sim == 'true'}
    if camera != 'none':
        backend = nodes.pop(0)
        own = [CONFIG + '/sim_backend.yaml'] if camera == 'sim' else []  # zed: camera.yaml
        assert (backend['executable'], backend['parameters']) == (BACKEND[camera][0],
                                                                  [*own, clock])
    tracker, costmap, manager, nearest = nodes
    assert tracker['parameters'] == [CONFIG + '/tracker.yaml', clock]
    assert nearest['parameters'] == [CONFIG + '/nearest_obstacle.yaml', clock]
    assert (costmap['namespace'], costmap['name']) == ('perception', 'costmap')
    hull = yaml.safe_load((ROOT / 'config/hull.yaml').read_text())
    bow, stern, y = hull['bow'], -hull['stern'], hull['half_width']
    assert costmap['parameters'] == [CONFIG + '/perception_costmap.yaml', {
        'footprint': str([[bow, y], [bow, -y], [stern, -y], [stern, y]]),
        'footprint_padding': hull['padding']}, clock]
    assert (manager['namespace'], manager['name']) == ('perception',
                                                       'lifecycle_manager_perception')
    assert manager['parameters'] == [{'autostart': True, 'node_names': ['costmap'],
                                      'bond_timeout': 0.0, **clock}]


@pytest.mark.parametrize('camera,sim', [('sim', 'true'), ('zed', 'false')])
def test_enabled_fusion_feeds_the_tracker_and_moves_the_depth_term(run, monkeypatch, camera, sim):
    fusion(run.module, monkeypatch, enabled=True)
    nodes = run(camera=camera, lidar='none', sim=sim)
    assert executables(nodes)[-5:-3] == ['lidar_fusion', 'object_tracker']
    fused, tracker = named(nodes, 'lidar_fusion'), named(nodes, 'object_tracker')
    assert fused['respawn'] is True and fused['respawn_delay'] == 1.0
    assert tracker['parameters'][-1] == {
        'observations_topic': '/perception/observations_fused', 'meas.depth_term': False}
    extra = fused['parameters'][2:]
    assert [p['points_transport'] for p in extra] == (['direct'] if sim == 'true' else [])


def test_fusion_is_off_by_default(run):
    assert 'lidar_fusion' not in executables(run(camera='sim', lidar='sim', sim='true'))


# Camera

def test_the_wrapper_include_loads_our_files_and_publishes_no_pose_tf(run):
    (include,) = [r for r in run(lidar='none') if r.kind == 'IncludeLaunchDescription']
    assert packages([include]) == {'zed_wrapper'}
    name, passed = included(include)
    assert name == 'zed_camera.launch.py'
    for key in ('publish_tf', 'publish_map_tf', 'publish_imu_tf'):
        assert passed[key] == 'false'
    assert passed['publish_urdf'] == 'true'
    assert (passed['camera_name'], passed['camera_model']) == ('front', 'zed2i')
    assert passed['ros_params_override_path'] == CONFIG + '/zed_boat.yaml'
    assert passed['custom_object_detection_config_path'] == (
        CONFIG + '/custom_object_detection.yaml')
    # The model from this package's share, never a home path.
    assert passed['param_overrides'] == (
        'object_detection.custom_onnx_file:=' + CONFIG + '/epoch60.onnx')


def wrapper_arguments():
    try:
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('zed_wrapper'),
                      'launch/zed_camera.launch.py').read_text()
    except Exception:
        pytest.skip('zed_wrapper is not installed')
    return set(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)'", source))


def test_the_wrapper_include_passes_only_arguments_the_wrapper_declares(run):
    (include,) = [r for r in run(lidar='none') if r.kind == 'IncludeLaunchDescription']
    assert set(included(include)[1]) <= wrapper_arguments()


def test_the_camera_scope_uses_the_large_frame_profile_as_is_without_an_address(run):
    records = run(lidar='none')
    camera = [r for r in records if r.kind == 'IncludeLaunchDescription'
              or r.get('package') == 'topic_tools']
    assert len(camera) == 3
    profile = CONFIG + '/camera_dds.xml'
    assert all(r.env == {'FASTRTPS_DEFAULT_PROFILES_FILE': profile,
                         'FASTDDS_DEFAULT_PROFILES_FILE': profile} for r in camera)
    assert all(r.env == {} for r in records if r not in camera)   # the scope ends with the camera


def test_the_camera_profile_whitelists_the_boat_address_and_loopback(run, monkeypatch):
    monkeypatch.setenv('ASTRO_DDS_ADDRESS', '192.168.8.20')
    env = named(run(lidar='none'), 'front_info_relay').env
    pinned = env['FASTRTPS_DEFAULT_PROFILES_FILE']
    assert env['FASTDDS_DEFAULT_PROFILES_FILE'] == pinned != CONFIG + '/camera_dds.xml'
    ns = '{%s}' % run.module.FASTDDS_NS
    udp = [d for d in ET.parse(pinned).iter(ns + 'transport_descriptor')
           if d.findtext(ns + 'type') == 'UDPv4']
    assert udp
    for descriptor in udp:
        assert [a.text for a in descriptor.iter(ns + 'address')] == ['192.168.8.20', '127.0.0.1']
        assert descriptor.findtext(ns + 'maxMessageSize') == '1400'
    Path(pinned).unlink()


def test_the_boat_relays_are_named_lazy_and_carry_no_raw_images(run):
    rate = resolve_camera().preview_rate
    relays = {r['name']: (r['executable'], r['arguments'], r['parameters'])
              for r in run(lidar='none') if r.get('package') == 'topic_tools'}
    assert relays == {
        'front_info_relay': ('relay', [
            '/front/zed_node/rgb/color/rect/camera_info', '/front_camera/camera_info'],
            [{'lazy': True}]),
        'front_preview_throttle': ('throttle', [
            'messages', '/front/zed_node/rgb/color/rect/image/compressed', str(rate),
            '/front_camera/preview/compressed'], [{'lazy': True}]),
    }


def test_mapping_adds_lazy_raw_relays_named_after_the_camera(run):
    relays = {r['name']: r for r in run(lidar='none', mapping='true', camera_name='rear')
              if r.get('package') == 'topic_tools'}
    assert set(relays) == {'rear_info_relay', 'rear_preview_throttle', 'rear_rgb_relay',
                           'rear_depth_relay'}
    # RTAB-Map subscribes to sensor_msgs/Image: the raw image, never the compressed one.
    assert relays['rear_rgb_relay']['arguments'] == [
        '/rear/zed_node/rgb/color/rect/image', '/rear_camera/rgb/image']
    assert relays['rear_depth_relay']['arguments'] == [
        '/rear/zed_node/depth/depth_registered', '/rear_camera/depth/image']
    assert all(r['parameters'] == [{'lazy': True}] for r in relays.values())


@pytest.mark.parametrize('name', ['front', 'rear'])
@pytest.mark.parametrize('sim', ['true', 'false'])
def test_the_relays_feed_the_rtabmap_remaps(run, harness, name, sim):  # noqa: F811
    produced = {r['name']: r['arguments'][-1]
                for r in run(lidar='none', mapping='true', camera_name=name)
                if r.get('package') == 'topic_tools'}
    rtabmap = named(harness.run('navigation.launch.py', nav2='false', mapping='true', sim=sim,
                                camera_name=name), 'rtabmap')
    targets = dict(rtabmap['remappings'])
    assert targets['/rgb/camera_info'] == produced[f'{name}_info_relay']
    assert targets['/rgb/image'] == produced[f'{name}_rgb_relay']
    assert targets['/depth/image'] == produced[f'{name}_depth_relay']


@pytest.mark.parametrize('stage,hz', [('package', None), ('deployment', 4), ('operator', 5),
                                      ('alias', 6)])
def test_each_camera_layer_reaches_the_throttle(run, tmp_path, stage, hz):
    arguments = {}
    for key, value in (('deployment', 4), ('operator', 5)):
        path = tmp_path / f'{key}.yaml'
        path.write_text(f'tune: {{preview_rate: {value}}}\n')
        if ['package', 'deployment', 'operator', 'alias'].index(stage) >= (
                1 if key == 'deployment' else 2):
            arguments[f'camera_{"startup" if key == "operator" else key}_file'] = str(path)
    if stage == 'alias':
        arguments['preview_rate'] = '6'
    throttle = named(run(lidar='none', **arguments), 'front_preview_throttle')
    expected = resolve_camera().preview_rate if hz is None else float(hz)
    assert throttle['arguments'][2] == str(expected)


@pytest.mark.parametrize('raw', [
    'preview_rate: false', 'preview_rate: .nan', 'preview_rate: 2\npreview_rate: 3',
    'unknown: 2', 'missing'])
def test_an_invalid_camera_layer_builds_nothing_and_writes_no_dds_profile(
        run, tmp_path, monkeypatch, raw):
    monkeypatch.setenv('ASTRO_DDS_ADDRESS', '192.168.8.20')
    path = tmp_path / 'bad.yaml'
    if raw != 'missing':
        path.write_text(raw)
    written = []
    monkeypatch.setattr(tempfile, 'mkstemp', lambda *a, **kw: written.append(1) or pytest.fail(
        'wrote a DDS profile'))
    with pytest.raises((ValueError, OSError)):
        run(lidar='none', camera_startup_file=str(path), preview_rate='9')
    assert run.harness.built == [] and written == []


def test_the_zed_backend_reads_no_file_of_its_own(run):
    """zed_backend reads camera.yaml backend:, so it gets only the clock."""
    assert named(run(lidar='none'), 'zed_backend')['parameters'] == [{'use_sim_time': False}]


# Lidar

def physical(records):
    return [r for r in records if r.get('package') in ('velodyne_driver', 'velodyne_pointcloud')]


def test_the_velodyne_pair_shares_the_resolved_model_frame_and_address(run, share):
    resolved = resolve_lidar(share=share, overrides={'velodyne_ip': '10.2.3.4'})
    records = run(camera='none', velodyne_ip='10.2.3.4')
    driver, transform = physical(records)
    assert driver['parameters'] == [resolved['driver']]
    assert transform['parameters'] == [resolved['transform']]
    assert driver['parameters'][0]['device_ip'] == '10.2.3.4'
    assert driver['parameters'][0]['model'] == transform['parameters'][0]['model'] == 'VLP16'
    assert driver['parameters'][0]['frame_id'] == transform['parameters'][0]['frame_id'] == (
        'lidar_link')
    large = CONFIG + '/fastdds_large_data.xml'
    for node in (driver, transform):   # TF and odometry keep the default transport
        assert node['additional_env'] == {'FASTRTPS_DEFAULT_PROFILES_FILE': large,
                                          'FASTDDS_DEFAULT_PROFILES_FILE': large}
    assert transform['remappings'] == [('velodyne_points', '/points')]
    assert 'additional_env' not in named(records, 'lidar_relay')
    assert named(records, 'nearby')['parameters'] == [{'use_sim_time': False}]


def test_the_lidar_frame_is_sensor_local_but_a_required_frame_refuses(run, tmp_path):
    """The robot's URDF cannot silently diverge from the cloud's frame."""
    selected = overlay(tmp_path, {'frame': 'custom_lidar'})
    own = physical(run(camera='none', lidar_params_file=selected))
    assert all(n['parameters'][0]['frame_id'] == 'custom_lidar' for n in own)
    run.harness.built.clear()
    with pytest.raises(ValueError, match='URDF'):
        run(camera='none', lidar_params_file=selected, lidar_required_frame='lidar_link')
    assert run.harness.built == []
    assert '<child link="lidar_link"/>' in (
        ROOT / 'description/blastoise/blastoise.xacro').read_text()


@pytest.mark.parametrize('selection', ['missing', 'invalid', 'duplicate'])
def test_an_invalid_lidar_selection_fails_before_any_node(run, tmp_path, selection):
    path = tmp_path / selection
    if selection != 'missing':
        path.write_text('driver: []' if selection == 'invalid' else 'driver: {rpm: 1, rpm: 2}')
    with pytest.raises((OSError, ValueError)):
        run(camera='none', lidar_params_file=str(path))
    assert run.harness.built == []


@pytest.mark.parametrize('patch', [
    {'endpoint': {'port': 0}}, {'endpoint': {'ip': 'invalid'}}, {'model': 'HDL64'},
    {'frame': ''}, {'calibration': {'file': 17}}])
def test_a_masked_invalid_deployment_layer_fails_before_any_node(run, tmp_path, patch):
    with pytest.raises(ValueError):
        run(camera='none', lidar_deployment_file=overlay(tmp_path, patch),
            lidar_params_file=str(ROOT / 'config/lidar.yaml'))
    assert run.harness.built == []


def test_a_corrupt_calibration_fails_before_any_node(run, share):
    (share('velodyne_pointcloud') / 'params/VLP16db.yaml').write_text('lasers: []')
    with pytest.raises(ValueError):
        run(camera='none')
    assert run.harness.built == []


@pytest.mark.parametrize('lidar', ['sim', 'none'])
def test_an_unused_lidar_selection_is_never_read(run, lidar):
    run.module.resolve_lidar = lambda *args, **kwargs: pytest.fail('resolved the unused lidar')
    assert executables(run(camera='none', lidar=lidar, lidar_params_file='/missing'))


def test_every_lidar_and_camera_layer_argument_defaults_empty(harness):  # noqa: F811
    declared = harness.declared('perception.launch.py')
    assert all(declared[key] == '' for key in (
        'lidar_params_file', 'lidar_deployment_file', 'lidar_required_frame', 'velodyne_ip',
        'velodyne_port', 'camera_deployment_file', 'camera_startup_file', 'preview_rate'))
    assert {key: declared[key] for key in ('camera', 'lidar', 'sim', 'mapping', 'camera_name',
                                           'camera_model')} == {
        'camera': 'zed', 'lidar': 'velodyne', 'sim': 'false', 'mapping': 'false',
        'camera_name': 'front', 'camera_model': 'zed2i'}
