"""mount_check recovers known mount tilts and taped-buoy offsets from synthetic data."""
import math

from mhseals_nav.mount_check import (ERROR, horizon, load_config, normal_roll_pitch, OK,
                                     taped_buoy, UNOBSERVABLE, water_tilt)
from mhseals_nav.robot_geometry import rpy_matrix
import numpy as np
import pytest

CFG = load_config()


def water_scan(roll, pitch, z=-0.45, seed=0):
    """Return level water at z seen through a lidar mount really tilted (roll, pitch) deg.

    The URDF takes the mount as unrotated, so the points come out in the frame it believes.
    """
    rng = np.random.default_rng(seed)
    r, a = rng.uniform(2.5, 7.5, 600), rng.uniform(-1.2, 1.2, 600)
    world = np.stack([r * np.cos(a), r * np.sin(a), z + rng.normal(0, 0.01, 600)], axis=1)
    mount = rpy_matrix(math.radians(roll), math.radians(pitch), 0.)
    return world @ mount   # = mount.T @ p: true-level points expressed in the tilted frame


@pytest.mark.parametrize('roll, pitch', [(0, 0), (3, -2), (-5, 5)])
def test_water_plane_recovers_lidar_tilt(roll, pitch):
    est_roll, est_pitch, z = water_tilt(water_scan(roll, pitch), CFG)
    assert est_roll == pytest.approx(roll, abs=0.15)
    assert est_pitch == pytest.approx(pitch, abs=0.15)


def test_a_narrow_strip_of_water_is_unobservable_not_level():
    scan = water_scan(2, 2)
    strip = scan[np.abs(scan[:, 1]) < 0.1]
    assert water_tilt(strip, CFG) is None


def test_normal_roll_pitch_sign_matches_urdf_rpy():
    for roll, pitch in [(4, 0), (0, 4), (-2, 3)]:
        up_seen = rpy_matrix(math.radians(roll), math.radians(pitch), 0.).T[:, 2]
        assert normal_roll_pitch(up_seen) == pytest.approx((roll, pitch), abs=1e-6)


def render_horizon(pitch, roll, w=640, h=360, f=500.):
    """Render sky over water from a camera (camera_link axes) pitched and rolled (deg)."""
    R = rpy_matrix(math.radians(roll), math.radians(pitch), 0.)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    # optical ray (right, down, forward) -> camera_link (forward, left, up) -> level frame
    ray = np.stack([np.full_like(u, f), -(u - w / 2), -(v - h / 2)], axis=-1)
    up = ray @ R.T
    return np.where(up[..., 2] > 0, 200., 60.), f, w / 2, h / 2


@pytest.mark.parametrize('pitch, roll', [(0, 0), (3, 0), (-2, 4), (5, -3)])
def test_horizon_recovers_camera_pitch_and_roll(pitch, roll):
    gray, f, cx, cy = render_horizon(pitch, roll)
    est_pitch, est_roll = horizon(gray, f, f, cx, cy, CFG)
    assert est_pitch == pytest.approx(pitch, abs=0.15)
    assert est_roll == pytest.approx(roll, abs=0.15)


def test_no_horizon_is_unobservable():
    rng = np.random.default_rng(1)
    assert horizon(rng.uniform(0, 255, (360, 640)), 500., 500., 320., 180., CFG) is None


def buoy_face(centre, n=40):
    """Return lidar points on the near face of a 0.25 m buoy at centre (base_link x, y)."""
    c = np.asarray(centre, float)
    toward = -c / np.linalg.norm(c)
    a = np.linspace(-1.0, 1.0, n) + math.atan2(toward[1], toward[0])
    return np.stack([c[0] + 0.125 * np.cos(a), c[1] + 0.125 * np.sin(a), np.zeros(n)], axis=1)


def test_taped_buoy_passes_where_the_lidar_agrees():
    dr, db, v = taped_buoy(buoy_face((6.0, 1.0)), (6.0, 1.0), CFG)
    assert v == OK and abs(dr) < 0.03 and abs(db) < 0.2


def test_taped_buoy_sees_a_whole_rail_yaw_the_online_check_cannot():
    """Catch the rail turned 2 deg with both sensors: the buoy reads 2 deg off its tape."""
    yaw = math.radians(2)
    seen = buoy_face((6.0, 1.0)) @ rpy_matrix(0., 0., yaw)   # true points in the turned frame
    dr, db, v = taped_buoy(seen, (6.0, 1.0), CFG)
    assert v == ERROR and db == pytest.approx(-2.0, abs=0.3)


def test_taped_buoy_without_returns_is_unobservable():
    assert taped_buoy(buoy_face((12.0, 0.0)), (6.0, 1.0), CFG)[2] == UNOBSERVABLE
