"""The boat URDF holds every sensor frame the config names, each once, on one tree from base_link.

robot_state_publisher (odometry part, on the ODROID) publishes these; the EKFs, navsat_transform
and the lidar consumers look them up. A frame the config names but the URDF lacks fails only at
runtime (dropped measurements), so this pins the names to the config that uses them.
"""
from pathlib import Path

from mhseals_nav.robot_geometry import RobotGeometry
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).parents[1]
GEOMETRY = RobotGeometry.load(ROOT / 'description/blastoise/blastoise.xacro')


def chain(frame):
    """Links from frame up to base_link."""
    return GEOMETRY.chain(frame)


def offset(frame):
    """base_link -> frame translation, asserting every joint on the chain is unrotated."""
    for link in chain(frame)[:-1]:
        assert np.allclose(GEOMETRY.joints[link][2], np.eye(3)), \
            f'{link} is rotated; the sensor axes are no longer base_link axes'
    return tuple(GEOMETRY.pose(frame)[1])


def sensor_frames():
    """Each frame name the stack's config gives a sensor, with where it comes from."""
    lidar = yaml.safe_load((ROOT / 'config/lidar.yaml').read_text())['lidar']['frame']
    adapter = (ROOT / 'mhseals_nav/mavros_odometry.py').read_text()
    assert "declare_parameter('gps_frame', 'gps_link')" in adapter
    return {'lidar.yaml lidar.frame': lidar, 'odometry adapter gps_frame': 'gps_link',
            'FCU IMU (documentation)': 'imu_link', 'ZED camera mount': 'front_camera_link'}


@pytest.mark.parametrize('use, frame', sensor_frames().items())
def test_every_configured_sensor_frame_hangs_off_base_link(use, frame):
    assert chain(frame)[-1] == 'base_link', use


def test_sensor_axes_are_base_link_axes():
    """Keep every sensor frame on REP-103 base_link axes (x forward, y left, z up).

    MAVROS publishes the FCU IMU already in base_link, and the lidar deskew uses base_link's
    twist for lidar_link.
    """
    for frame in ('lidar_link', 'gps_link', 'imu_link'):
        offset(frame)   # asserts every joint on the chain is unrotated


def test_lidar_lever_arm_left_out_of_deskew_is_small():
    x, y, _ = offset('lidar_link')
    # lidar_relay.deskew ignores w x r: under 3 cm per 0.1 s scan at 1 rad/s.
    assert (x * x + y * y) ** 0.5 * 1.0 * 0.1 < 0.03
