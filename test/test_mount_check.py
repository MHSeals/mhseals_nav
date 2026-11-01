"""mount_check recovers known mount tilts and taped-buoy offsets from synthetic data."""
import math

from mhseals_nav.mount_check import (attitude_at, CONFIG, ERROR, horizon, load_config,
                                     normal_roll_pitch, OK, taped_buoy, UNOBSERVABLE, verdict,
                                     WARN, water_tilt)
from mhseals_nav.robot_geometry import rpy_matrix
import numpy as np
import pytest
import yaml

CFG = load_config()
DEG = math.radians(1)


def seen_through(world, roll, pitch):
    """Level-world points expressed in a lidar mount really tilted (roll, pitch) rad.

    The URDF takes the mount as unrotated, so the points come out in the frame it believes.
    """
    return world @ rpy_matrix(roll, pitch, 0.)   # = mount.T @ p


def water(z=-0.2, n=600, seed=0, azimuth=(-math.pi, math.pi)):
    """Level water at z, returns all around the boat between 2.5 and 7.5 m."""
    rng = np.random.default_rng(seed)
    r, a = rng.uniform(2.5, 7.5, n), rng.uniform(*azimuth, n)
    return np.stack([r * np.cos(a), r * np.sin(a), z + rng.normal(0, 0.01, n)], axis=1)


def deck(z=-0.1, list_roll=-DEG, n=1500, seed=1):
    """Return a floating dock alongside to port (y 1-3 m, x -6-6 m) listing about its length."""
    rng = np.random.default_rng(seed)
    x, y = rng.uniform(-6, 6, n), rng.uniform(1.0, 3.0, n)
    return np.stack([x, y, z + (y - 2.0) * math.tan(list_roll) + rng.normal(0, 0.005, n)], 1)


@pytest.mark.parametrize('roll, pitch', [(0, 0), (3, -2), (-5, 5)])
def test_water_plane_recovers_lidar_tilt(roll, pitch):
    est_roll, est_pitch, z = water_tilt(seen_through(water(), roll * DEG, pitch * DEG),
                                        CFG['water'])
    assert est_roll == pytest.approx(roll * DEG, abs=0.15 * DEG)
    assert est_pitch == pytest.approx(pitch * DEG, abs=0.15 * DEG)
    assert z == pytest.approx(-0.2, abs=0.02)


def test_a_dense_deck_over_sparse_water_reads_the_water():
    """A deck listing -1 deg would cancel a real +1 deg lidar roll; the lower water plane wins."""
    scan = seen_through(np.vstack([deck(), water(n=250)]), DEG, 0.)
    roll, pitch, z = water_tilt(scan, CFG['water'])
    assert roll == pytest.approx(DEG, abs=0.15 * DEG) and z == pytest.approx(-0.2, abs=0.02)


def test_a_deck_alone_is_unobservable_not_level():
    assert water_tilt(seen_through(deck(), DEG, 0.), CFG['water']) is None


def test_water_on_one_side_only_is_unobservable():
    assert water_tilt(water(azimuth=(0.1, 3.0)), CFG['water']) is None


def test_a_plane_off_the_waterline_is_not_water():
    assert water_tilt(water(z=-0.8), CFG['water']) is None
    assert water_tilt(water(z=-0.02), CFG['water']) is None    # at the hull tops


def test_a_taped_waterline_narrows_the_band():
    assert water_tilt(water(z=-0.2), dict(CFG['water'], waterline=-0.18)) is not None
    assert water_tilt(water(z=-0.2), dict(CFG['water'], waterline=-0.1)) is None


def test_a_narrow_strip_of_water_is_unobservable_not_level():
    scan = seen_through(water(), 2 * DEG, 2 * DEG)
    assert water_tilt(scan[np.abs(scan[:, 1]) < 0.1], CFG['water']) is None


def test_normal_roll_pitch_sign_matches_urdf_rpy():
    for roll, pitch in [(4, 0), (0, 4), (-2, 3)]:
        up_seen = rpy_matrix(roll * DEG, pitch * DEG, 0.).T[:, 2]
        assert normal_roll_pitch(up_seen) == pytest.approx((roll * DEG, pitch * DEG), abs=1e-9)


def render(pitch, roll, w=640, h=360, f=500., shore=0.0, treeline=None, chop=0.0, mirror=0.0,
           seed=0):
    """Render a grey image from a camera (camera_link axes) pitched and rolled (rad).

    Water below the shore edge (elevation ``shore`` rad), optionally a textured treeline
    ``treeline`` rad above it, sky above that. ``chop`` is the water's texture (grey levels).
    ``mirror`` > 0 reflects the treeline in the water, darkened, mixed with that share of the
    reflection (1 glassy; the rest noise).
    """
    rng = np.random.default_rng(seed)
    R = rpy_matrix(roll, pitch, 0.)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    # optical ray (right, down, forward) -> camera_link (forward, left, up) -> level frame
    ray = np.stack([np.full_like(u, f), -(u - w / 2), -(v - h / 2)], axis=-1) @ R.T
    elevation = np.arcsin(ray[..., 2] / np.linalg.norm(ray, axis=-1))
    image = np.where(elevation > shore, 220., 110. + rng.normal(0, 1.0 + chop, (h, w)))

    texture = rng.uniform(10, 70, (w // 4 + 1, 400))

    def trees(e):   # treeline texture in 4 px x 0.006 rad blocks, so it can be mirrored
        return texture[(u // 4).astype(int), np.clip(np.round(e / 0.006).astype(int), -200, 199)]
    if treeline:
        band = (elevation > shore) & (elevation < shore + treeline)
        image[band] = trees(elevation)[band]
        reflection = (elevation <= shore) & (elevation > shore - treeline)
        if mirror:
            mixed = (mirror * trees(2 * shore - elevation)
                     + (1 - mirror) * rng.uniform(10, 70, (h, w)))
            image[reflection] = 0.8 * mixed[reflection]
    return image, f, w / 2, h / 2


@pytest.mark.parametrize('pitch, roll', [(0, 0), (3, 0), (-2, 4), (5, -3)])
def test_horizon_recovers_camera_pitch_and_roll(pitch, roll):
    gray, f, cx, cy = render(pitch * DEG, roll * DEG)
    est_pitch, est_roll = horizon(gray, f, f, cx, cy, CFG['horizon'])
    assert est_pitch == pytest.approx(pitch * DEG, abs=0.15 * DEG)
    assert est_roll == pytest.approx(roll * DEG, abs=0.15 * DEG)


@pytest.mark.parametrize('pitch, roll', [(0, 0), (2, -3)])
def test_a_treeline_above_the_horizon_does_not_bias_pitch(pitch, roll):
    """A 6 m treeline at 150 m stands 2.3 deg above the water edge; the edge is the horizon."""
    gray, f, cx, cy = render(pitch * DEG, roll * DEG, treeline=2.3 * DEG)
    est_pitch, est_roll = horizon(gray, f, f, cx, cy, CFG['horizon'])
    assert est_pitch == pytest.approx(pitch * DEG, abs=0.15 * DEG)
    assert est_roll == pytest.approx(roll * DEG, abs=0.15 * DEG)


def test_a_known_shore_elevation_is_taken_off_the_pitch():
    gray, f, cx, cy = render(DEG, 0., shore=-1.2 * DEG, treeline=2.3 * DEG)
    known = dict(CFG['horizon'], shore_elevation=-1.2 * DEG)
    assert horizon(gray, f, f, cx, cy, known)[0] == pytest.approx(DEG, abs=0.15 * DEG)
    unknown = horizon(gray, f, f, cx, cy, CFG['horizon'])[0]
    assert unknown == pytest.approx(-0.2 * DEG, abs=0.15 * DEG)


@pytest.mark.parametrize('roll', [0, 3])
def test_a_mirrored_treeline_reads_the_water_edge_between(roll):
    """Glassy water: the reflection's bottom sits 2.3 deg under the water edge; level reads 0."""
    gray, f, cx, cy = render(0., roll * DEG, treeline=2.3 * DEG, mirror=1.0)
    est_pitch, est_roll = horizon(gray, f, f, cx, cy, CFG['horizon'])
    assert est_pitch == pytest.approx(0., abs=0.15 * DEG)
    assert est_roll == pytest.approx(roll * DEG, abs=0.15 * DEG)


def test_a_camera_pitched_over_a_mirror_never_reads_ok():
    """2 deg nose-down over a mirrored treeline: the mount error is seen, not cancelled."""
    gray, f, cx, cy = render(2 * DEG, 0., treeline=2.3 * DEG, mirror=1.0)
    estimate = horizon(gray, f, f, cx, cy, CFG['horizon'])
    assert estimate is not None
    assert verdict([(estimate[1], estimate[0])], CFG['verdict'])[2] != OK
    assert estimate[0] == pytest.approx(2 * DEG, abs=0.15 * DEG)


def test_a_half_mirrored_treeline_is_unobservable():
    gray, f, cx, cy = render(0., 0., treeline=2.3 * DEG, mirror=0.3)
    assert horizon(gray, f, f, cx, cy, CFG['horizon']) is None


def test_a_treeline_over_choppy_water_is_unobservable():
    """No smooth water under any edge: ambiguous, never the treeline."""
    gray, f, cx, cy = render(DEG, 0., treeline=2.3 * DEG, chop=25.0)
    assert horizon(gray, f, f, cx, cy, CFG['horizon']) is None


def test_no_horizon_is_unobservable():
    rng = np.random.default_rng(1)
    assert horizon(rng.uniform(0, 255, (360, 640)), 500., 500., 320., 180., CFG['horizon']) is None


def buoy_face(centre, half_angle=math.pi / 2, n=40, radius=0.125):
    """Lidar points on the near face of a round object at centre (base_link x, y)."""
    c = np.asarray(centre, float)
    a = np.linspace(-half_angle, half_angle, n) + math.atan2(-c[1], -c[0])
    return np.stack([c[0] + radius * np.cos(a), c[1] + radius * np.sin(a), np.zeros(n)], axis=1)


@pytest.mark.parametrize('half_angle', [math.pi / 2, 1.0])
def test_taped_buoy_passes_where_the_lidar_agrees(half_angle):
    """The circle fit takes out the near-face bias (a half circle's centroid is 0.64 r short)."""
    dr, db, v = taped_buoy(buoy_face((6.0, 1.0), half_angle), (6.0, 1.0), CFG['buoy'])
    assert v == OK and abs(dr) < 0.005 and abs(db) < 0.05 * DEG


def test_taped_buoy_sees_a_whole_rail_yaw_the_online_check_cannot():
    """Catch the rail turned 2 deg with both sensors: the buoy reads 2 deg off its tape."""
    seen = buoy_face((6.0, 1.0)) @ rpy_matrix(0., 0., 2 * DEG)   # true points in the turned frame
    dr, db, v = taped_buoy(seen, (6.0, 1.0), CFG['buoy'])
    assert v == ERROR and db == pytest.approx(-2 * DEG, abs=0.3 * DEG)


def test_taped_buoy_without_returns_is_unobservable():
    assert taped_buoy(buoy_face((12.0, 0.0)), (6.0, 1.0), CFG['buoy'])[2] == UNOBSERVABLE


def test_taped_buoy_ignores_a_pole_beside_it_and_refuses_a_pole_or_dock_alone():
    pole = buoy_face((6.0, 1.6), radius=0.03, n=6)
    dock = np.stack([np.linspace(5.0, 7.0, 100), np.full(100, 1.5), np.zeros(100)], axis=1)
    dr, db, v = taped_buoy(np.vstack([buoy_face((6.0, 1.0)), pole]), (6.0, 1.0), CFG['buoy'])
    assert v == OK and abs(dr) < 0.005
    assert taped_buoy(pole, (6.0, 1.6), CFG['buoy'])[2] == UNOBSERVABLE
    assert taped_buoy(dock, (6.0, 1.3), CFG['buoy'])[2] == UNOBSERVABLE


def test_verdict_warns_on_scan_to_scan_spread():
    steady = [(0.001, -0.001)] * 10
    noisy = [(0.001 + 0.02 * (-1) ** i, 0.0) for i in range(10)]
    assert verdict(steady, CFG['verdict'])[2] == OK
    assert verdict(noisy, CFG['verdict'])[2] == WARN
    assert verdict([(2 * DEG, 0.)] * 5, CFG['verdict'])[2] == ERROR


def test_attitude_is_matched_by_stamp():
    stamps, attitudes = np.array([0.0, 0.1, 0.2]), ['a', 'b', 'c']
    assert attitude_at(stamps, attitudes, 0.13, 0.05) == 'b'
    assert attitude_at(stamps, attitudes, 0.27, 0.05) is None
    assert attitude_at(np.array([]), [], 0.0, 0.05) is None


def test_config_is_strict(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    for change, message in [
            (lambda c: c['water'].update(range_deg=[2, 8]), 'keys differ'),
            (lambda c: c['verdict'].update(tilt_warn='0.01'), 'tilt_warn'),
            (lambda c: c['water'].update(min_points=60.0), 'min_points'),
            (lambda c: c['buoy'].update(radius=-0.1), 'radius'),
            (lambda c: c['water'].update(range=[8.0, 2.0]), 'range'),
            (lambda c: c['water'].update(waterline='-0.2'), 'waterline')]:
        bad = yaml.safe_load(yaml.safe_dump(data))
        change(bad['mount_check'])
        path = tmp_path / 'bad.yaml'
        path.write_text(yaml.safe_dump(bad))
        with pytest.raises(ValueError, match=message):
            load_config(path)
