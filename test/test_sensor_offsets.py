"""The sensor mounts in the blastoise URDF follow its xacro properties (REP-103: x forward)."""
import re

from mhseals_nav.robot_geometry import description_path, expand, RobotGeometry
import numpy as np
import pytest

DESCRIPTION = description_path().read_text()
P = {name: float(value) for name, value in re.findall(
    r'<xacro:property name="(\w+)" value="(-?[\d.]+)"/>', DESCRIPTION)}
# The mounts composed into base_link from the xacro's properties alone (no tilts), so swapping
# gps_x and imu_x in the xacro is the only edit that moves them. The camera is the ZED wrapper's
# camera_link: lens (body centre + half the depth) - zed_lens_x forward, - half the height.
MEASURED = {
    'gps_link': (P['gps_x'], P['gps_y'], P['datum_z'] + P['gps_z']),
    'imu_link': (P['imu_x'], P['imu_y'], P['datum_z'] + P['imu_z']),
    'lidar_link': (P['lidar_x'], P['lidar_y'], P['datum_z'] + P['lidar_z']),
    'front_camera_link': (P['camera_x'] + P['zed_depth'] / 2 - P['zed_lens_x'], P['camera_y'],
                          P['datum_z'] + P['camera_z'] - P['zed_height'] / 2),
}
GEOMETRY = RobotGeometry.load()


@pytest.mark.parametrize('frame', MEASURED)
def test_sensor_origin_matches_the_xacro_properties(frame):
    R, t = GEOMETRY.pose(frame)
    assert tuple(t) == pytest.approx(MEASURED[frame], abs=1e-6)
    assert np.allclose(R, np.eye(3))          # tilts come from calibration, not tape


def test_base_link_is_the_com_in_x_and_y():
    """The FCU's body origin is base_link (INS_POS), so base_link's inertial origin stays there."""
    node = expand(description_path().read_text()).find("link[@name='base_link']/inertial/origin")
    assert [float(v) for v in node.get('xyz').split()[:2]] == [0.0, 0.0]
