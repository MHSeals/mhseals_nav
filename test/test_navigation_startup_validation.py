"""Actual validation and startup factory failures, without ROS imports."""
import ast
from pathlib import Path

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
        validated_merge_limits(base, patch)


@pytest.mark.parametrize('text', ['[', '[]', 'false', 'a: 1\na: 2', 'a: &x {}\nb: *x', '<<: {}'])
def test_invalid_existing_file_is_not_absent(tmp_path, text):
    path = tmp_path / 'overlay.yaml'
    path.write_text(text)
    with pytest.raises((ValueError, yaml.YAMLError)):
        load_overlay(path, strict=True)


def test_absent_optional_file_and_valid_overlay(base, tmp_path):
    assert load_overlay(tmp_path / 'absent', strict=True) == {}
    patch = {'controller_server': {'ros__parameters': {'controller_frequency': 20}}}
    result = validated_merge_limits(base, patch)
    assert result['controller_server']['ros__parameters']['FollowPath']['model_dt'] == .05
    with pytest.raises(ValueError):
        load_overlay(tmp_path, strict=True)  # existing but unreadable as a file


def test_actual_navigation_factory_fails_before_node(base, tmp_path):
    base_path = tmp_path / 'base.yaml'
    base_path.write_text(yaml.safe_dump(base))
    overlay = tmp_path / 'bad.yaml'
    overlay.write_text('controller_server: []')
    ns = inert_navigation_factory()
    ns['Node'] = lambda **kw: pytest.fail('Node constructed before validation')
    with pytest.raises(ValueError):
        ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': str(overlay)})


@pytest.mark.parametrize('patch', [
    {'waypoint_follower': {'ros__parameters': {
        'wait_at_waypoint': {'waypoint_pause_duration': -1}}}},
    {'velocity_smoother': {'ros__parameters': {'deadband_velocity': [-1.0, 0.0, 0.0]}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'PathAngleCritic': {'mode': -1}}}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'PathAngleCritic': {'mode': 3}}}}},
])
def test_known_zero_default_domains_rejected(base, patch):
    with pytest.raises(ValueError):
        validated_merge_limits(base, patch)


@pytest.mark.parametrize('vector', [[0, 0, 0], [0, 0.0, 0]])
def test_emitted_deadband_vector_has_selected_base_float_type(base, vector):
    result = validated_merge_limits(base, {'velocity_smoother': {'ros__parameters': {
        'deadband_velocity': vector}}})
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
    result = validated_merge_limits(base, {'Extension': {'signed_offset': -1.0}})
    assert result['Extension']['signed_offset'] == -1.0


def inert_navigation_factory():
    """Compile the real factory with only inert launch dependencies."""
    source = ROOT / 'launch/navigation.launch.py'
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == 'navigation_nodes')

    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context):
            defaults = {'sim': 'true', 'use_lidar': 'true'}
            return context.get(self.name, defaults.get(self.name))
    import os
    import tempfile
    import sys
    import types
    actions = types.ModuleType('launch.actions')
    actions.RegisterEventHandler = lambda *a, **kw: None
    events = types.ModuleType('launch.event_handlers')
    events.OnShutdown = lambda **kw: None
    import shutil
    from types import SimpleNamespace
    share = Path(tempfile.mkdtemp())
    shutil.copytree(ROOT / 'mhseals_nav/behavior_trees', share / 'behavior_trees')
    (share / 'config').mkdir()
    shutil.copyfile(ROOT / 'config/nav2_ceilings.yaml', share / 'config/nav2_ceilings.yaml')
    (share / 'launch').mkdir()
    shutil.copyfile(ROOT / 'launch/navigation.launch.py', share / 'launch/navigation.launch.py')
    ns = {'LaunchConfiguration': Configuration,
          'os': os,
          'yaml': yaml,
          'tempfile': tempfile,
          'FindPackageShare': lambda name: SimpleNamespace(perform=lambda _: str(share)),
          'PathJoinSubstitution': lambda parts: parts,
          'RewrittenYaml': lambda **kw: kw,
          'ParameterValue': lambda *a, **kw: a,
          'Node': lambda **kw: kw,
          'OpaqueFunction': lambda **kw: kw}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), ns)
    factory = ns['navigation_nodes']
    from unittest.mock import patch

    def run(context):
        with patch.dict(sys.modules, {
                'launch.actions': actions, 'launch.event_handlers': events}):
            return factory(context)

    ns['share'] = share
    ns['navigation_nodes'] = run
    return ns


def test_absent_optional_overlay_preserves_custom_controller_base(base, tmp_path):
    controller = base['controller_server']['ros__parameters']
    controller['CustomController'] = controller.pop('FollowPath')
    controller['controller_plugins'] = ['CustomController']
    path = tmp_path / 'custom.yaml'
    path.write_text(yaml.safe_dump(base))
    ns = inert_navigation_factory()
    nodes = ns['navigation_nodes']({'params_file': str(path),
                                   'limits_file': str(tmp_path / 'absent.yaml')})
    assert len(nodes) == 7
    emitted = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    emitted_controller = emitted['controller_server']['ros__parameters']
    assert emitted_controller['CustomController'] == controller['CustomController']
    assert 'FollowPath' not in emitted['controller_server']['ros__parameters']
    # A real overlay on this selected base must not require a FollowPath mapping.
    merged = validated_merge_limits(base, {'controller_server': {'ros__parameters': {
        'CustomController': {'vx_max': 0.3}}}})
    assert merged['controller_server']['ros__parameters']['CustomController']['vx_max'] == 0.3
    assert 'FollowPath' not in merged['controller_server']['ros__parameters']


@pytest.mark.parametrize('patch', [
    {'waypoint_follower': {'ros__parameters': {
        'wait_at_waypoint': {'waypoint_pause_duration': -1}}}},
    {'velocity_smoother': {'ros__parameters': {'deadband_velocity': [-1.0, 0.0, 0.0]}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'PathAngleCritic': {'mode': -1}}}}},
    None,
])
def test_review_reproductions_fail_before_node(base, tmp_path, patch):
    import sys
    import types
    from unittest.mock import patch as patch_modules
    base_path = tmp_path / 'base.yaml'
    base_path.write_text(yaml.safe_dump(base))
    overlay = tmp_path / 'overlay.yaml'
    overlay.write_text(yaml.safe_dump(patch))
    ns = inert_navigation_factory()
    ns['Node'] = lambda **kw: pytest.fail('constructed node with invalid requested configuration')
    actions = types.ModuleType('launch.actions')
    actions.RegisterEventHandler = lambda *a, **kw: None
    events = types.ModuleType('launch.event_handlers')
    events.OnShutdown = lambda **kw: None
    with patch_modules.dict(sys.modules, {
            'launch.actions': actions, 'launch.event_handlers': events}):
        with pytest.raises(ValueError):
            ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': str(overlay)})


def test_actual_factory_consumes_final_projection(base, tmp_path):
    params = base['controller_server']['ros__parameters']
    params['enable_stamped_cmd_vel'] = True
    params['publish_tf'] = False
    base['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = '/chosen/tree.xml'
    path = tmp_path / 'base.yaml'
    path.write_text(yaml.safe_dump(base))
    context = {'params_file': str(path), 'limits_file': '', 'sim': 'true',
               'use_lidar': 'false'}
    ns = inert_navigation_factory()
    nodes = ns['navigation_nodes'](context)
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
    tree = ns['share'] / 'behavior_trees/nav_to_pose.xml'
    assert effective['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(tree)
    assert effective['lifecycle_manager_navigation']['ros__parameters']['use_sim_time'] is True


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return path


def test_factory_accepts_an_overlay_within_the_ceilings(base, tmp_path):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    overlay = write(tmp_path, 'ok.yaml', 'controller_server: {ros__parameters: {FollowPath: {wz_max: 0.5, vx_min: -0.05}}}')
    from mhseals_nav.startup_projection import compile_navigation
    ns = inert_navigation_factory()
    assert ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': str(overlay)})
    result = compile_navigation(base_path, overlay, sim='true', use_lidar='true', share=ns['share'])
    follow = result['controller_server']['ros__parameters']['FollowPath']
    assert follow['wz_max'] == 0.5 and follow['vx_min'] == -0.05


@pytest.mark.parametrize('typo', ['vx_max: 5.0', 'vx_max: 1.0e+6', 'vx_min: -5.0', 'ax_min: -0.7', 'ay_min: -2.0'])
def test_factory_refuses_an_overlay_beyond_the_ceilings(base, tmp_path, typo):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    overlay = write(tmp_path, 'typo.yaml', 'controller_server: {ros__parameters: {FollowPath: {%s}}}' % typo)
    ns = inert_navigation_factory()
    ns['Node'] = lambda **kw: pytest.fail('Node constructed past a ceiling')
    with pytest.raises(ValueError, match='ceiling'):
        ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': str(overlay)})


def test_legacy_merge_still_drops_beyond_the_ceiling(base):
    from mhseals_nav.navigation_config import merge_limits
    limits = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_min': -5.0}}}}
    merged = merge_limits(base, limits, {'vx_min': 0.2})
    follow = merged['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_min'] == base['controller_server']['ros__parameters']['FollowPath']['vx_min']


def test_missing_ceilings_fail_closed(base, tmp_path):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    ns = inert_navigation_factory()
    (ns['share'] / 'config/nav2_ceilings.yaml').unlink()
    ns['Node'] = lambda **kw: pytest.fail('Node constructed without ceilings')
    with pytest.raises(FileNotFoundError):
        ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': ''})


@pytest.mark.parametrize('text', ['vx_max: .nan', 'vx_max: .inf', 'vx_max: 0', 'vx_max: -0.8',
                                  "vx_max: '0.8'", 'vx_max: true', '[0.8]'])
def test_invalid_ceilings_fail_at_load(base, tmp_path, text):
    base_path = write(tmp_path, 'base.yaml', yaml.safe_dump(base))
    ns = inert_navigation_factory()
    (ns['share'] / 'config/nav2_ceilings.yaml').write_text(text)
    ns['Node'] = lambda **kw: pytest.fail('Node constructed with an invalid ceiling')
    with pytest.raises(ValueError):
        ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': ''})


def test_a_nan_ceiling_passed_in_directly_is_refused(base):
    with pytest.raises(ValueError, match='positive finite'):
        validated_merge_limits(base, {}, {'vx_max': float('nan')})
