"""robot_geometry reads the description as xacro does, so ROS-free code sees TF's tree."""
import math

from mhseals_nav import robot_geometry as rg
import numpy as np
import pytest
from test_sensor_offsets import P

TEXT = rg.description_path().read_text()


def test_the_minimal_expander_matches_xacro_on_every_fixed_joint():
    pytest.importorskip('xacro')
    full = rg.RobotGeometry(rg.expand(TEXT))
    lite = rg.RobotGeometry(rg._expand_lite(TEXT, {}))
    for child, (parent, kind, R, t) in lite.joints.items():
        assert full.joints[child][:2] == (parent, kind)
        assert np.allclose(full.joints[child][2], R) and np.allclose(full.joints[child][3], t)


def test_a_property_override_moves_only_its_mount():
    base = rg.RobotGeometry.load()
    moved = rg.RobotGeometry.load(properties={'lidar_z': 0.5,
                                              'rail_rpy': f'0 {math.radians(3)} 0'})
    assert moved.pose('lidar_link')[1][2] == pytest.approx(
        base.pose('sensor_mount_link')[1][2] + 0.5 * math.cos(math.radians(3)))
    assert np.allclose(moved.pose('gps_link')[1], base.pose('gps_link')[1])


def test_an_unknown_property_is_refused():
    with pytest.raises(KeyError, match='lidar_q'):
        rg.RobotGeometry.load(properties={'lidar_q': 1.0})


def test_pose_composes_rotations_in_urdf_order():
    g = rg.RobotGeometry.load(properties={'box_rpy': f'0 0 {math.pi / 2}'})
    # yawed about the datum
    assert g.pose('gps_link')[1][:2] == pytest.approx([-P['gps_y'], P['gps_x']])


def test_matrix_quaternion_round_trips_through_lidar_fusion():
    from mhseals_nav.lidar_fusion import quat_matrix
    for angles in [(0.1, -0.2, 0.3), (3.0, 0.4, -2.5), (0.0, 0.0, math.pi)]:
        R = rg.rpy_matrix(*angles)
        assert np.allclose(quat_matrix(*rg.matrix_quaternion(R)), R)
