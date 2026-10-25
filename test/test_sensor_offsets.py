"""Pin the measured GPS and IMU offsets in the blastoise URDF (REP-103: x forward, y left)."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

URDF = Path(__file__).parents[1] / 'description/blastoise/blastoise.xacro'

# Taped from the CoM on 2026-10-25: both 5 in to starboard, antenna 2 in forward, Cube 2 in aft.
MEASURED = {
    'gps_link_joint': (0.051, -0.127, 0.23439),
    'imu_link_joint': (-0.051, -0.127, 0.0),
}


def origin(joint):
    """Read the xyz of a fixed joint's origin from the xacro."""
    node = ET.parse(URDF).getroot().find(f"joint[@name='{joint}']/origin")
    return tuple(float(v) for v in node.get('xyz').split())


@pytest.mark.parametrize('joint', MEASURED)
def test_sensor_origin_matches_the_measurement(joint):
    assert origin(joint) == pytest.approx(MEASURED[joint])


def test_signs_antenna_forward_cube_aft_both_starboard():
    gps, imu = origin('gps_link_joint'), origin('imu_link_joint')
    assert gps[0] > 0 > imu[0]
    assert gps[1] < 0 and imu[1] < 0


def test_base_link_is_the_com_in_x_and_y():
    """The offsets are from the CoM, so base_link's inertial origin must stay there."""
    node = ET.parse(URDF).getroot().find("link[@name='base_link']/inertial/origin")
    assert [float(v) for v in node.get('xyz').split()[:2]] == [0.0, 0.0]
