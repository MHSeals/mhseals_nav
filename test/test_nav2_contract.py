"""Static guardrails for the Jazzy boat profile; not a hydrodynamic test."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_rpp_and_command_limits():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    rpp = controller['FollowPath']
    assert controller['odom_topic'] == '/odom/local'
    assert controller['progress_checker_plugins'] == ['progress_checker']
    assert rpp['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert rpp['use_collision_detection']
    assert not rpp['allow_reversing']
    smoother = config['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == rpp['desired_linear_vel']
    assert smoother['max_velocity'][2] == rpp['rotate_to_heading_angular_vel']
    assert smoother['min_velocity'][0] == 0.0
    assert smoother['max_velocity'][1] == 0.0
    assert smoother['velocity_timeout'] > 0


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


def test_recovery_trees_do_not_command_blind_motion():
    for name in ['nav_to_pose.xml', 'nav_replan_recov.xml']:
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
    assert "'object_tracker'" not in (ROOT / 'launch/odom.launch.py').read_text()
    assert "'object_tracker'" not in launch
    assert not (ROOT / 'launch/slam_nav.launch.py').exists()
    slam = (ROOT / 'launch/slam.launch.py').read_text()
    assert "package='rtabmap_slam'" in slam
    assert 'navigation.launch.py' not in slam
    robot = (ROOT / 'launch/robot.launch.py').read_text()
    for child in ('navigation', 'slam', 'objects'):
        assert f"'{child}.launch.py'" in robot
