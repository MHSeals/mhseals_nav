"""Pin the measured sensor mounts in the blastoise URDF (REP-103: x forward, y left, z up)."""
from mhseals_nav.robot_geometry import description_path, expand, RobotGeometry
import numpy as np
import pytest

# The owner's tape of 2026-11-01 from the mount datum (electrical box bottom centre = hull tops,
# z -0.0126 in base_link), composed into base_link. The camera is the ZED wrapper's camera_link:
# lens (body centre + half the 43.1 mm depth) + 0.01 m forward, - half the 30.3 mm height.
MEASURED = {
    'gps_link': (0.0699, -0.1207, 0.0954),
    'imu_link': (-0.0572, -0.1016, 0.0954),
    'lidar_link': (0.2921, 0.0, 0.2351),
    'front_camera_link': (0.4001 + 0.0431 / 2 + 0.01, 0.0, 0.1397 - 0.0303 / 2 - 0.0126),
}
GEOMETRY = RobotGeometry.load()


@pytest.mark.parametrize('frame', MEASURED)
def test_sensor_origin_matches_the_measurement(frame):
    R, t = GEOMETRY.pose(frame)
    assert tuple(t) == pytest.approx(MEASURED[frame], abs=1e-6)
    assert np.allclose(R, np.eye(3))          # tilts come from calibration, not tape


def test_signs_antenna_forward_cube_aft_both_starboard():
    gps, imu = GEOMETRY.pose('gps_link')[1], GEOMETRY.pose('imu_link')[1]
    assert gps[0] > 0 > imu[0]
    assert gps[1] < 0 and imu[1] < 0


def test_base_link_is_the_com_in_x_and_y():
    """The FCU's body origin is base_link (INS_POS), so base_link's inertial origin stays there."""
    node = expand(description_path().read_text()).find("link[@name='base_link']/inertial/origin")
    assert [float(v) for v in node.get('xyz').split()[:2]] == [0.0, 0.0]
