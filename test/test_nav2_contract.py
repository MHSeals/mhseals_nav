"""Static guardrails for the Jazzy boat profile; not a hydrodynamic test."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_omni_mppi_and_command_limits():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    assert controller['odom_topic'] == '/odom/local'
    assert controller['progress_checker_plugins'] == ['progress_checker']
    assert mppi['primary_controller'] == 'nav2_mppi_controller::MPPIController'
    assert mppi['motion_model'] == 'Omni'
    # MPPI advances its plan one model_dt per control cycle.
    assert mppi['model_dt'] == 1.0 / controller['controller_frequency']
    assert mppi['prune_distance'] >= mppi['vx_max'] * mppi['time_steps'] * mppi['model_dt']
    assert 'CostCritic' in mppi['critics'] and mppi['CostCritic']['consider_footprint']
    # The camera looks forward: prefer facing along the path.
    assert mppi['PathAngleCritic']['mode'] == 0 and 'PreferForwardCritic' in mppi['critics']
    # Sim: MPPI alone reverses to a goal behind the boat; the shim must turn it first.
    assert mppi['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    # Below ~1.6 the shim re-engages mid-route and near the goal (stop-and-turn cycles).
    assert 1.6 <= mppi['angular_dist_threshold'] <= 2.2
    assert mppi['rotate_to_heading_once'] is True
    assert mppi['rotate_to_heading_angular_vel'] <= mppi['wz_max']
    assert mppi['max_angular_accel'] <= mppi['az_max']
    assert mppi['rotate_to_goal_heading'] is False
    smoother = config['velocity_smoother']['ros__parameters']
    # The smoother passes everything MPPI may command, sway included, and no more.
    assert smoother['max_velocity'] == [mppi['vx_max'], mppi['vy_max'], mppi['wz_max']]
    assert smoother['min_velocity'] == [mppi['vx_min'], -mppi['vy_max'], -mppi['wz_max']]
    assert smoother['velocity_timeout'] > 0
    assert smoother['max_decel'][1] == mppi['ay_min']
    assert mppi['TwirlingCritic']['cost_weight'] > 0
    assert all(mppi[f'{axis}_std'] <= mppi[f'{axis}_max'] for axis in ('vx', 'vy', 'wz'))
    # Nav2's 20 ms default timed out acknowledging compute_path_to_pose in the field.
    assert config['bt_navigator']['ros__parameters']['default_server_timeout'] >= 100
    assert 'smoother_server' not in config  # no behavior tree uses SmoothPath
    assert controller['precise_goal_checker']['plugin'] == 'nav2_controller::StoppedGoalChecker'
    assert (controller['precise_goal_checker']['xy_goal_tolerance']
            < controller['general_goal_checker']['xy_goal_tolerance'])


def test_inflation_leaves_a_six_foot_gate_open():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    gate = 6 * 0.3048
    for name in ['local_costmap', 'global_costmap']:
        params = config[name][name]['ros__parameters']
        footprint = yaml.safe_load(params['footprint'])
        half_beam = max(abs(y) for _, y in footprint) + params['footprint_padding']
        # Lethal space ends at the inscribed radius; the hull must fit between two buoys.
        assert 2 * half_beam < gate - 0.2
        # The gate's middle may cost something, but well short of the inscribed cost.
        inflation = params['inflation_layer']
        middle = 252 * math.exp(-inflation['cost_scaling_factor'] * (gate / 2 - half_beam))
        assert middle < 128
        # MPPI checks the whole footprint only once the cost says a corner may touch;
        # that cost is read at the circumscribed radius, so inflation must reach it.
        circumscribed = max(math.hypot(abs(x) + params['footprint_padding'],
                                       abs(y) + params['footprint_padding'])
                            for x, y in footprint)
        assert params['inflation_layer']['inflation_radius'] >= circumscribed


def test_costmaps_do_not_require_slam_and_check_sensor_freshness():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    for name in ['local_costmap', 'global_costmap']:
        params = config[name][name]['ros__parameters']
        assert params['rolling_window']
        assert 'static_layer' not in params['plugins']
        footprint = yaml.safe_load(params['footprint'])
        assert len(footprint) >= 3
        assert any(x > 0 for x, _ in footprint)
        assert any(x < 0 for x, _ in footprint)
        # Obstacles come only from the Orin's perception costmap, kept graded.
        assert params['plugins'] == ['perception_layer', 'inflation_layer']
        layer = params['perception_layer']
        assert layer['plugin'] == 'nav2_costmap_2d::StaticLayer'
        assert layer['map_topic'] == '/perception/costmap'
        assert layer['subscribe_to_updates'] is True
        assert layer['footprint_clearing_enabled'] is False
        assert params['trinary_costmap'] is False and params['use_maximum'] is False
        assert 'trinary_costmap' not in layer   # StaticLayer reads it from the costmap
        if name == 'local_costmap':
            assert params['update_frequency'] >= 10.0
            assert params['publish_frequency'] >= 10.0
        else:
            assert params['update_frequency'] >= 1.0  # planning runs at 1 Hz
            assert params['publish_frequency'] >= 1.0


def test_recovery_trees_do_not_command_blind_motion():
    for name in ['nav_to_pose.xml', 'nav_replan_recov.xml', 'nav_precise.xml']:
        root = ET.parse(ROOT / 'mhseals_nav/behavior_trees' / name).getroot()
        assert not any(node.tag in ['Spin', 'BackUp', 'ClearEntireCostmap'] for node in root.iter())
        assert len(root.find('.//RecoveryNode')) == 2
        assert root.find('.//FollowPath') is not None


def test_launch_responsibilities_and_velocity_output():
    launch = (ROOT / 'launch/navigation.launch.py').read_text()
    assert "('cmd_vel_smoothed', 'cmd_vel')" in launch
    assert "('cmd_vel', 'cmd_vel_nav')" in launch
    assert "twist_converter" not in launch
    assert "enable_mavros_velocity" not in launch
    assert "LaunchConfiguration('use_lidar')" in launch
    assert "('received_global_plan', 'local_plan')" in launch
    assert "'object_tracker'" not in (ROOT / 'launch/odom.launch.py').read_text()
    assert "'object_tracker'" not in launch
    assert not (ROOT / 'launch/slam_nav.launch.py').exists()
    slam = (ROOT / 'launch/slam.launch.py').read_text()
    assert "package='rtabmap_slam'" in slam
    assert 'navigation.launch.py' not in slam
    robot = (ROOT / 'launch/robot.launch.py').read_text()
    for child in ('navigation', 'slam', 'objects'):
        assert f"'{child}.launch.py'" in robot


def test_rtabmap_uses_the_live_lidar_contract():
    config = yaml.safe_load((ROOT / 'config/rtabmap.yaml').read_text())
    params = config['rtabmap']['ros__parameters']
    assert params['subscribe_scan_cloud'] is True
    assert params['subscribe_scan'] is False
    assert params['subscribe_rgb'] is False
    assert params['subscribe_depth'] is False
    assert params['Rtabmap/DetectionRate'] == '10.0'
    assert '("/scan_cloud", "/points")' in (ROOT / 'launch/slam.launch.py').read_text()


def test_camera_relay_keeps_sensor_msgs_image_type():
    """RTAB-Map subscribes to sensor_msgs/Image, so its relay must read the raw image topic."""
    camera = (ROOT / 'launch/camera.launch.py').read_text()
    assert "[f'{zed}/rgb/color/rect/image', f'{out}/rgb/image']" in camera


def test_behavior_trees_reload_for_each_goal():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    assert config['bt_navigator']['ros__parameters']['always_reload_bt_xml'] is True


def test_packaged_params_are_already_consistent_with_derived():
    from mhseals_nav.navigation_config import derived
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    assert derived(config) == config


def _plugin_strings(node):
    """Yield every value under a 'plugin' or 'primary_controller' key."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ('plugin', 'primary_controller') and isinstance(value, str):
                yield value
            else:
                yield from _plugin_strings(value)
    elif isinstance(node, list):
        for value in node:
            yield from _plugin_strings(value)


def test_plugin_names_are_declared_by_the_installed_nav2():
    """A renamed class only fails when controller_server starts, so resolve it here."""
    ament_index = pytest.importorskip('ament_index_python.packages')
    declared = set()
    for package in ament_index.get_packages_with_prefixes():
        prefix = Path(ament_index.get_package_prefix(package))
        for xml in (prefix / 'share' / package).glob('*.xml'):
            try:
                root = ET.parse(xml).getroot()
            except ET.ParseError:
                continue
            for cls in root.iter('class'):
                declared.update(filter(None, (cls.get('name'), cls.get('type'))))
    if not any(name.startswith('nav2_') for name in declared):
        pytest.skip('Nav2 plugin XML is not installed')
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    missing = sorted(set(_plugin_strings(config)) - declared)
    assert not missing, f'not declared by any installed plugin XML: {missing}'


def test_package_xml_depends_on_rotation_shim():
    """The shim plugin ships in its own package, which the boat image must install."""
    xml = (Path(__file__).resolve().parents[1] / 'package.xml').read_text()
    assert '<exec_depend>nav2_rotation_shim_controller</exec_depend>' in xml


@pytest.mark.parametrize('use_lidar', [True, False])
def test_factory_emits_installed_behavior_trees_and_lidar(use_lidar):
    from test_navigation_startup_validation import inert_navigation_factory

    factory = inert_navigation_factory()
    nodes = factory['navigation_nodes']({
        'params_file': str(ROOT / 'config/nav2_params.yaml'), 'limits_file': '',
        'sim': 'false', 'use_lidar': str(use_lidar).lower()})
    emitted = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    navigator = emitted['bt_navigator']['ros__parameters']
    expected = {'default_nav_to_pose_bt_xml': 'nav_to_pose.xml',
                'default_nav_through_poses_bt_xml': 'nav_replan_recov.xml'}
    for key, name in expected.items():
        artifact = Path(navigator[key])
        assert artifact == factory['share'] / 'behavior_trees' / name
        assert artifact.read_bytes() == (ROOT / 'mhseals_nav/behavior_trees' / name).read_bytes()
        tree = ET.fromstring(artifact.read_bytes())
        assert not any(node.tag in ['Spin', 'BackUp', 'ClearEntireCostmap']
                       for node in tree.iter())
        assert tree.find('.//FollowPath') is not None
    for name in ('local_costmap', 'global_costmap'):
        params = emitted[name][name]['ros__parameters']
        assert 'obstacle_layer' not in params    # lidar reaches B inside /perception/costmap
        assert params['perception_layer']['map_topic'] == '/perception/costmap'
