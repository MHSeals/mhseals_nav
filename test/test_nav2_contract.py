"""Static guardrails for the Jazzy boat profile; not a hydrodynamic test."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_omni_mppi_and_command_limits():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    assert controller['odom_topic'] == '/odom/local'
    assert controller['progress_checker_plugins'] == ['progress_checker']
    assert mppi['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert mppi['motion_model'] == 'Omni'
    # MPPI advances its plan one model_dt per control cycle.
    assert mppi['model_dt'] == 1.0 / controller['controller_frequency']
    assert mppi['prune_distance'] >= mppi['vx_max'] * mppi['time_steps'] * mppi['model_dt']
    assert 'CostCritic' in mppi['critics'] and mppi['CostCritic']['consider_footprint']
    # The camera looks forward: prefer facing along the path.
    assert mppi['PathAngleCritic']['mode'] == 0 and 'PreferForwardCritic' in mppi['critics']
    smoother = config['velocity_smoother']['ros__parameters']
    # The smoother passes everything MPPI may command, sway included, and no more.
    assert smoother['max_velocity'] == [mppi['vx_max'], mppi['vy_max'], mppi['wz_max']]
    assert smoother['min_velocity'] == [mppi['vx_min'], -mppi['vy_max'], -mppi['wz_max']]
    assert smoother['velocity_timeout'] > 0
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
        assert params['obstacle_layer']['pointcloud']['expected_update_rate'] > 0
        if name == 'local_costmap':
            assert params['update_frequency'] >= 10.0
            assert params['publish_frequency'] >= 10.0
        else:
            assert params['update_frequency'] >= 5.0
            assert params['publish_frequency'] >= 5.0


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
    assert 'default_nav_to_pose_bt_xml' in launch
    assert 'default_nav_through_poses_bt_xml' in launch
    assert 'obstacle_layer.enabled' in launch
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
    sensors = (ROOT / 'launch/sensors.launch.py').read_text()
    assert '/rgb/color/rect/image/compressed' not in sensors
    assert '/rgb/color/rect/image' in sensors
