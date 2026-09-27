"""Static guardrails for the Jazzy boat profile; not a hydrodynamic test."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_mppi_and_command_limits():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    assert controller['odom_topic'] == '/odom/local'
    assert controller['progress_checker_plugins'] == ['progress_checker']
    assert mppi['iteration_count'] == 1
    assert mppi['model_dt'] == 1 / controller['controller_frequency']
    assert 'iterations' not in mppi
    for critic in mppi['critics']:
        assert 'cost_weight' in mppi[critic]
    smoother = config['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [mppi['vx_max'], 0.0, mppi['wz_max']]
    assert smoother['max_accel'] == [mppi['ax_max'], 0.0, mppi['az_max']]


def test_costmaps_do_not_require_slam_and_check_sensor_freshness():
    config = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    for name in ['local_costmap', 'global_costmap']:
        params = config[name][name]['ros__parameters']
        assert params['rolling_window']
        assert 'static_layer' not in params['plugins']
        assert params['robot_radius'] >= 0.8
        assert params['obstacle_layer']['pointcloud']['expected_update_rate'] > 0


def test_recovery_trees_do_not_command_blind_motion():
    for name in ['nav_to_pose.xml', 'nav_replan_recov.xml']:
        root = ET.parse(ROOT / 'mhseals_nav/behavior_trees' / name).getroot()
        assert not any(node.tag in ['Spin', 'BackUp', 'ClearEntireCostmap'] for node in root.iter())
        assert len(root.find('.//RecoveryNode')) == 2
        assert root.find('.//FollowPath') is not None


def test_public_launch_resolves_server_file_and_keeps_actuation_opt_in():
    launch = (ROOT / 'launch/slam_nav.launch.py').read_text()
    assert "'navigation.launch.py'" in launch
    assert (ROOT / 'launch/navigation.launch.py').exists()
    assert "('cmd_vel_topic', '/nav/cmd_vel'" in launch
    assert "('enable_mavros_velocity', 'false'" in launch
    assert 'default_nav_to_pose_bt_xml' in launch
    assert 'default_nav_through_poses_bt_xml' in launch
