"""The FCU lever-arm proposal is the URDF's gps_link and imu_link in FRD, to the millimetre."""
from pathlib import Path
import subprocess
import sys

from mhseals_nav.robot_geometry import fcu_lever_arms
import pytest
from test_sensor_offsets import P

PARAMS = Path(__file__).parents[1] / 'config/fcu/blastoise_gnss_proposal.param'


def test_fcu_offsets_are_the_xacro_mounts_in_frd():
    z = P['datum_z']
    expected = {}
    for prefixes, kind in [(['GPS1_POS'], 'gps'), (['INS_POS1', 'INS_POS2', 'INS_POS3'], 'imu')]:
        for prefix in prefixes:
            expected.update({f'{prefix}_X': P[f'{kind}_x'], f'{prefix}_Y': -P[f'{kind}_y'],
                             f'{prefix}_Z': -(z + P[f'{kind}_z'])})
    assert fcu_lever_arms() == pytest.approx(expected, abs=6e-4)


def test_the_module_prints_the_proposal_lines():
    out = subprocess.run([sys.executable, '-m', 'mhseals_nav.robot_geometry'], check=True,
                         capture_output=True, text=True).stdout
    rows = dict(line.split(',') for line in out.splitlines())
    assert {k: float(v) for k, v in rows.items()} == pytest.approx(fcu_lever_arms(), abs=6e-4)


def test_the_param_file_does_not_restate_the_lever_arms():
    """The URDF is their one source: a second copy would go stale when the mounts change."""
    names = {line.split(',')[0] for line in PARAMS.read_text().splitlines()
             if line and not line.startswith('#')}
    assert not names & set(fcu_lever_arms())
