"""Container suite: the EKF holds still through a spin when the URDF's lever arm is right."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

PROBE = Path(__file__).parent / 'ekf_lever_arm_probe.py'


def probe(true=None, given=None):
    pytest.importorskip('rclpy')
    ament = pytest.importorskip('ament_index_python.packages')
    try:
        ament.get_package_prefix('robot_localization')
        ament.get_package_prefix('robot_state_publisher')
    except Exception:
        pytest.skip('robot_localization is not installed (container suite only)')
    env = dict(os.environ, ROS_DOMAIN_ID='231', PROBE_TRUE=true or '{}', PROBE_URDF=given or '{}')
    return subprocess.run([sys.executable, str(PROBE)], env=env, capture_output=True, text=True,
                          timeout=90)


def test_a_spin_leaves_the_fused_position_still_when_the_urdf_matches():
    done = probe()
    assert done.returncode == 0, done.stdout[-1500:] + done.stderr[-1500:]
    assert 'position spread' in done.stdout
