"""The FCU lever-arm proposal is the URDF's gps_link and imu_link in FRD, to the millimetre."""
from pathlib import Path

from mhseals_nav.robot_geometry import RobotGeometry
import pytest

PARAMS = Path(__file__).parents[1] / 'config/fcu/blastoise_gnss_proposal.param'


def params():
    rows = (line.split(',') for line in PARAMS.read_text().splitlines()
            if line and not line.startswith('#'))
    return {name: float(value) for name, value in rows}


@pytest.mark.parametrize('prefix, frame', [('GPS1_POS', 'gps_link'), ('INS_POS1', 'imu_link'),
                                           ('INS_POS2', 'imu_link'), ('INS_POS3', 'imu_link')])
def test_fcu_offsets_are_the_urdf_frames_in_frd(prefix, frame):
    x, y, z = RobotGeometry.load().pose(frame)[1]
    p = params()
    assert [p[f'{prefix}_{a}'] for a in 'XYZ'] == pytest.approx([x, -y, -z], abs=6e-4)
