"""The boat URDF holds every sensor frame the config names, each once, on one tree from base_link.

robot_state_publisher (odometry part, on the ODROID) publishes these; the EKFs, navsat_transform
and the lidar consumers look them up. A frame the config names but the URDF lacks fails only at
runtime (dropped measurements), so this pins the names to the config that uses them.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).parents[1]
URDF = ET.parse(ROOT / 'description/blastoise/blastoise.xacro').getroot()


def parents():
    """Map each child link to (parent link, xyz, rpy) for the literal fixed joints."""
    result = {}
    for joint in URDF.findall('joint'):
        child = joint.find('child').get('link')
        assert child not in result, f'{child} has two parents'
        origin = joint.find('origin')
        xyz = tuple(float(v) for v in origin.get('xyz', '0 0 0').split())
        rpy = tuple(float(v) for v in origin.get('rpy', '0 0 0').split())
        result[child] = (joint.find('parent').get('link'), xyz, rpy)
    return result


def chain(frame):
    """Links from frame up to base_link."""
    up, links = parents(), [frame]
    while links[-1] != 'base_link':
        assert links[-1] in up, f'{links[-1]} does not reach base_link'
        links.append(up[links[-1]][0])
    return links


def offset(frame):
    """base_link -> frame translation, valid while every joint on the chain is unrotated."""
    up, total = parents(), [0., 0., 0.]
    for link in chain(frame)[:-1]:
        _, xyz, rpy = up[link]
        assert rpy == (0., 0., 0.), f'{link} is rotated; sum translations no longer'
        total = [a + b for a, b in zip(total, xyz)]
    return tuple(total)


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
