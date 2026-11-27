"""Actual validation and startup factory failures, without ROS imports."""
from pathlib import Path
import shutil

from launch_harness import executables, harness, named  # noqa: F401
from mhseals_nav.startup_projection import TREES
from mhseals_nav.navigation_config import load_overlay, validated_merge_limits
import pytest
import yaml

ROOT = Path(__file__).parents[1]


@pytest.fixture
def base():
    return yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())


@pytest.mark.parametrize('patch', [
    {'velocity_smoother': {'ros__parameters': {'max_velocity': []}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'time_steps': -1}}}},
    {'controller_server': []},
    {'controller_server': {'ros__parameters': {'controller_frequency': float('nan')}}},
    {'velocity_smoother': {'ros__parameters': {'feedback': 'bad'}}},
    {'unknown_server': {}},
])
def test_invalid_requested_values_fail(base, patch):
    with pytest.raises(ValueError):
        validated_merge_limits(base, patch, {})


@pytest.mark.parametrize('text', ['[', '[]', 'false', 'a: 1\na: 2', 'a: &x {}\nb: *x', '<<: {}'])
def test_invalid_existing_file_is_not_absent(tmp_path, text):
    path = tmp_path / 'overlay.yaml'
    path.write_text(text)
    with pytest.raises((ValueError, yaml.YAMLError)):
        load_overlay(path, strict=True)


def test_absent_optional_file_and_valid_overlay(base, tmp_path):
    assert load_overlay(tmp_path / 'absent', strict=True) == {}
    patch = {'controller_server': {'ros__parameters': {'controller_frequency': 20}}}
    result = validated_merge_limits(base, patch, {})
    assert result['controller_server']['ros__parameters']['FollowPath']['model_dt'] == .05
    with pytest.raises(ValueError):
        load_overlay(tmp_path, strict=True)  # existing but unreadable as a file


def test_actual_navigation_factory_fails_before_node(base, tmp_path, navigation):
    base_path = tmp_path / 'base.yaml'
    base_path.write_text(yaml.safe_dump(base))
    state = tmp_path / 'hardware.yaml'
    state.write_text('characterization: {nav2_limits: {velocity: [0.3], acceleration: [0.3, 0.3, 0.3]}}')
    with pytest.raises(ValueError):
        navigation(params_file=str(base_path), hardware_config=str(state))
    assert navigation.harness.built == []


@pytest.mark.parametrize('patch', [
    {'waypoint_follower': {'ros__parameters': {
        'wait_at_waypoint': {'waypoint_pause_duration': -1}}}},
    {'velocity_smoother': {'ros__parameters': {'deadband_velocity': [-1.0, 0.0, 0.0]}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'PathAngleCritic': {'mode': -1}}}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'PathAngleCritic': {'mode': 3}}}}},
])
def test_known_zero_default_domains_rejected(base, patch):
    with pytest.raises(ValueError):
        validated_merge_limits(base, patch, {})


@pytest.mark.parametrize('vector', [[0, 0, 0], [0, 0.0, 0]])
def test_emitted_deadband_vector_has_selected_base_float_type(base, vector):
    result = validated_merge_limits(base, {'velocity_smoother': {'ros__parameters': {
        'deadband_velocity': vector}}}, {})
    emitted = yaml.safe_load(yaml.safe_dump(result))
    assert all(type(v) is float for v in
               emitted['velocity_smoother']['ros__parameters']['deadband_velocity'])


@pytest.mark.parametrize('text', ['null', '~', '---\nnull\n'])
def test_explicit_null_requested_file_rejected(tmp_path, text):
    path = tmp_path / 'null.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match='root must be a mapping'):
        load_overlay(path, strict=True)


@pytest.mark.parametrize('text', ['', '# first use\n'])
def test_empty_requested_file_is_supported(tmp_path, text):
    path = tmp_path / 'empty.yaml'
    path.write_text(text)
    assert load_overlay(path, strict=True) == {}


def test_unrelated_zero_default_can_remain_signed(base):
    base['Extension'] = {'signed_offset': 0.0}
    result = validated_merge_limits(base, {'Extension': {'signed_offset': -1.0}}, {})
    assert result['Extension']['signed_offset'] == -1.0


@pytest.fixture
def navigation(harness, monkeypatch, tmp_path):  # noqa: F811
    """Start navigation.launch.py against a package tree that layers.package_path answers with.

    The tree (package.xml beside config/) is navigation.share.
    """
    from mhseals_nav import layers
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    (share / 'package.xml').touch()
    for name in ('nav2_ceilings.yaml', 'hull.yaml', 'camera.yaml'):
        shutil.copyfile(ROOT / 'config' / name, share / 'config' / name)
    monkeypatch.setattr(layers, 'SOURCE', share)

    def start(**arguments):
        return harness.run('navigation.launch.py', **arguments)
    start.harness, start.share = harness, share
    return start


def test_absent_optional_overlay_preserves_custom_controller_base(base, tmp_path, navigation):
    controller = base['controller_server']['ros__parameters']
    controller['CustomController'] = controller.pop('FollowPath')
    controller['controller_plugins'] = ['CustomController']
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(base))
    nodes = navigation(params_file=str(path), hardware_config=str(tmp_path / 'absent.yaml'))
    assert len(nodes) == 7
    emitted = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    emitted_controller = emitted['controller_server']['ros__parameters']
    # Its plugin is the MPPI shim, so it still takes the shim limits from its own wz_max/az_max.
    custom = controller['CustomController']
    assert emitted_controller['CustomController'] == {
        **custom, 'rotate_to_heading_angular_vel': custom['wz_max'], 'max_angular_accel': custom['az_max']}
    assert 'FollowPath' not in emitted['controller_server']['ros__parameters']
    # A real overlay on this selected base must not require a FollowPath mapping.
    merged = validated_merge_limits(base, {'controller_server': {'ros__parameters': {
        'CustomController': {'vx_max': 0.3}}}}, {})
    assert merged['controller_server']['ros__parameters']['CustomController']['vx_max'] == 0.3
    assert 'FollowPath' not in merged['controller_server']['ros__parameters']


def test_actual_factory_consumes_final_projection(base, tmp_path, navigation):
    params = base['controller_server']['ros__parameters']
    params['enable_stamped_cmd_vel'] = True
    params['publish_tf'] = False
    base['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = '/chosen/tree.xml'
    path = tmp_path / 'base.yaml'
    path.write_text(yaml.safe_dump(base))
    nodes = navigation(params_file=str(path), hardware_config=str(tmp_path / 'absent.yaml'),
                       sim='true')
    effective = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    assert all(yaml.safe_load(Path(n['parameters'][0]).read_text()) == effective for n in nodes)
    emitted = effective['controller_server']['ros__parameters']
    assert emitted['enable_stamped_cmd_vel'] is False
    assert emitted['use_sim_time'] is True
    assert emitted['publish_tf'] is False
    for name in ('local_costmap', 'global_costmap'):
        costmap = effective[name][name]['ros__parameters']
        assert costmap['use_sim_time'] is True
        assert 'obstacle_layer' not in costmap
    tree = TREES / 'nav_to_pose.xml'   # beside the module, as run_task's precise tree
    assert effective['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(tree)
    assert effective['lifecycle_manager_navigation']['ros__parameters']['use_sim_time'] is True


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return path


def nav2_limits(velocity, acceleration=(0.3, 0.3, 0.5)):
    return yaml.safe_dump({'characterization': {'nav2_limits': {'velocity': list(velocity),
                                                                'acceleration': list(acceleration)}}})


def test_factory_accepts_nav2_limits_within_the_ceilings(base, tmp_path, navigation):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    state = write(tmp_path, 'hardware.yaml', nav2_limits((0.3, 0.1, 0.2)))
    from mhseals_nav.startup_projection import compile_navigation
    assert navigation(params_file=str(base_path), hardware_config=str(state))
    result = compile_navigation(base_path, state, sim='true', share=navigation.share)
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['wz_max'] == 0.2 and follow['vx_max'] == 0.3


@pytest.mark.parametrize('velocity, acceleration', [((5.0, 0.2, 0.3), (0.3, 0.3, 0.5)),
                                                    ((1.0e6, 0.2, 0.3), (0.3, 0.3, 0.5)),
                                                    ((0.4, 0.2, 0.3), (0.7, 0.3, 0.5)),
                                                    ((0.4, 0.2, 0.3), (0.3, 2.0, 0.5))])
def test_factory_clips_nav2_limits_beyond_the_ceilings(
        base, tmp_path, velocity, acceleration, capsys, navigation):
    """The boat is never locked out by a measurement: Nav2 starts at the ceiling, with one warning."""
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    state = write(tmp_path, 'hardware.yaml', nav2_limits(velocity, acceleration))
    nodes = navigation(params_file=str(base_path), hardware_config=str(state))
    emitted = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    follow = emitted['controller_server']['ros__parameters']['FollowPath']
    ceilings = yaml.safe_load((navigation.share / 'config/nav2_ceilings.yaml').read_text())
    assert all(abs(follow[key]) <= ceilings[key] for key in ceilings if key in follow)
    assert len([line for line in capsys.readouterr().err.splitlines() if 'clipped' in line]) == 1


def test_legacy_merge_still_drops_beyond_the_ceiling(base):
    from mhseals_nav.navigation_config import merge_limits
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_min': -5.0}}}}
    merged = merge_limits(base, limits, {'vx_min': 0.2})
    follow = merged['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_min'] == base['controller_server']['ros__parameters']['FollowPath']['vx_min']


def test_the_packaged_ceilings_bound_an_overlay(base):
    from mhseals_nav.navigation_config import load_ceilings, merge_limits
    from mhseals_nav.startup_projection import ceilings_path
    ceilings = load_ceilings(ceilings_path())

    def follow(vx_min):
        overlay = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_min': vx_min}}}}
        merged = merge_limits(base, overlay, ceilings)
        return merged['controller_server']['ros__parameters']['FollowPath']
    assert follow(-0.15)['vx_min'] == -0.1      # within the 0.2 ceiling, then capped by the base
    assert follow(-0.25)['vx_min'] == -0.1      # beyond the ceiling: dropped
    with pytest.raises(ValueError, match='ceiling'):
        validated_merge_limits(base, {'controller_server': {'ros__parameters': {
            'FollowPath': {'vx_min': -0.25}}}}, ceilings)
    with pytest.raises(ValueError, match='mapping'):
        validated_merge_limits(base, {}, None)                       # no ceilings is not unbounded


def test_missing_ceilings_fail_closed(base, tmp_path, navigation):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    (navigation.share / 'config/nav2_ceilings.yaml').unlink()
    with pytest.raises(FileNotFoundError):
        navigation(params_file=str(base_path), hardware_config='')
    assert navigation.harness.built == []


@pytest.mark.parametrize('text', ['vx_max: .nan', 'vx_max: .inf', 'vx_max: 0', 'vx_max: -0.8',
                                  "vx_max: '0.8'", 'vx_max: true', '[0.8]'])
def test_invalid_ceilings_fail_at_load(base, tmp_path, text, navigation):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    (navigation.share / 'config/nav2_ceilings.yaml').write_text(text)
    with pytest.raises(ValueError):
        navigation(params_file=str(base_path), hardware_config='')
    assert navigation.harness.built == []


def test_a_nan_ceiling_passed_in_directly_is_refused(base):
    with pytest.raises(ValueError, match='positive finite'):
        validated_merge_limits(base, {}, {'vx_max': float('nan')})


# navigation.launch.py's switches and mission.launch.py

def test_nav2_owns_the_velocity_output_and_the_plan_rviz_reads(harness):  # noqa: F811
    nodes = harness.run('navigation.launch.py')
    assert executables(nodes) == ['controller_server', 'planner_server', 'behavior_server',
                                  'bt_navigator', 'waypoint_follower', 'velocity_smoother',
                                  'lifecycle_manager']
    assert named(nodes, 'controller_server')['remappings'] == [
        ('cmd_vel', 'cmd_vel_nav'), ('received_global_plan', 'local_plan')]
    assert named(nodes, 'behavior_server')['remappings'] == [('cmd_vel', 'cmd_vel_nav')]
    assert named(nodes, 'velocity_smoother')['remappings'] == [
        ('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    # The lidar reaches Nav2 only through /perception/costmap; the tracker is perception's.
    assert not {'object_tracker', 'rtabmap'} & set(executables(nodes))


@pytest.mark.parametrize('sim', ['false', 'true', '0', 'True'])
def test_mapping_is_rtabmap_on_the_lidar_with_a_boolean_clock(harness, sim):  # noqa: F811
    def navigation(**arguments):
        return harness.run('navigation.launch.py', **arguments)
    (rtabmap,) = navigation(nav2='false', mapping='true', sim=sim)
    assert (rtabmap['package'], rtabmap['executable']) == ('rtabmap_slam', 'rtabmap')
    assert rtabmap['parameters'][1] == {'use_sim_time': sim in ('true', 'True')}
    assert rtabmap['parameters'][0].endswith('/config/rtabmap.yaml')
    assert dict(rtabmap['remappings'])['/scan_cloud'] == '/points'
    assert executables(navigation(mapping='true'))[-1] == 'rtabmap'
    assert navigation(nav2='false') == []


@pytest.mark.parametrize('arguments', [
    {'nav2': 'yes'}, {'nav2': 'false', 'mapping': 'on'}, {'sim': ''}])
def test_an_invalid_navigation_switch_fails_before_any_node(harness, arguments):  # noqa: F811
    with pytest.raises(ValueError):
        harness.run('navigation.launch.py', **arguments)
    assert harness.built == []


def test_mission_starts_nothing_without_a_task(harness):  # noqa: F811
    assert harness.run('mission.launch.py') == []


@pytest.mark.parametrize('sim', ['true', 'false'])
def test_mission_runs_the_task_with_its_parameters_and_clock(harness, sim):  # noqa: F811
    (task,) = harness.run('mission.launch.py', task='nav_channel', sim=sim)
    assert (task['package'], task['executable']) == ('mhseals_nav', 'run_task')
    assert task['arguments'] == ['--task', 'nav_channel']
    assert task['parameters'] == [{'use_sim_time': sim == 'true'}]
    (chosen,) = harness.run('mission.launch.py', task='nav_channel', params_file='/x/t.yaml')
    assert chosen['arguments'] == ['--task', 'nav_channel', '--params', '/x/t.yaml']
