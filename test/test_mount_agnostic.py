"""Arrangement-agnostic perception and GNSS: move a sensor, update only the URDF, nothing breaks.

Every case draws the world from a "true" robot description and runs the stack's pure code with
the description it was given (robot_geometry, the same file robot_state_publisher publishes):
- matched (true == given) for the measured mounts and for each sensor moved by tens of cm and
  turned a few degrees: tracks land on the buoys in the world frame, lidar fusion ranges them,
  and the GNSS lever arm cancels during a spin;
- mismatched (the sensor moved, the URDF did not): the damage is the size the geometry
  predicts, and the stack's own checks see it (the lidar-camera extrinsic estimate alarms, the
  spin draws the lever-arm circle).
"""
import math
from types import SimpleNamespace as NS

from mhseals_nav import lidar_fusion as lf
from mhseals_nav.robot_geometry import RobotGeometry, rpy_matrix
import numpy as np
import pytest
import test_perception_metamorphic as pm    # installs duck msgs on a bare host
import test_sensor_offsets as sensor_offsets
import yaml

from mhseals_nav import sim_backend as sb  # noqa: E402, I100
from mhseals_nav.track_filter import MOTION_STATIC, TrackFilter  # noqa: E402, I100

DEG = math.pi / 180
CFG = yaml.safe_load((pm.ROOT / 'config' / 'lidar_fusion.yaml').read_text())[
    '/**']['ros__parameters']
CAMERA, LIDAR, GPS = 'front_camera_link', 'lidar_link', 'gps_link'
WATER_Z = -0.45                # water plane in base_link for these scenes (hull tops 0.43 up)
IMAGE = (1280, 720)
K = (640.0, 640.0, 640.0, 360.0)
RINGS = np.radians(np.arange(-15, 16, 2))
AZ = np.radians(np.arange(-180, 180, 0.2))


def rpy(roll=0.0, pitch=0.0, yaw=0.0):
    return f'{roll * DEG} {pitch * DEG} {yaw * DEG}'


# Each arrangement overrides blastoise.xacro's mount properties (m, deg).
ARRANGEMENTS = {
    'measured': {},
    'lidar_moved': {'lidar_x': 0.05, 'lidar_y': 0.25, 'lidar_z': 0.45,
                    'lidar_rpy': rpy(2.0, -3.0, 4.0)},
    'camera_moved': {'camera_x': 0.10, 'camera_y': -0.30, 'camera_z': 0.35,
                     'camera_rpy': rpy(-2.0, 3.0, -5.0)},
    # The rail with both sensors on it, 0.29 m aft and 0.2 m to port, pitched and yawed.
    'rail_moved': {'rail_x': 0.0, 'rail_y': 0.2, 'rail_rpy': rpy(0.0, 4.0, 3.0),
                   'lidar_x': 0.0, 'lidar_y': 0.2, 'camera_x': 0.108, 'camera_y': 0.2},
    'gps_moved': {'gps_x': -0.45, 'gps_y': 0.30, 'gps_z': 0.40, 'box_rpy': rpy(3.0, -2.0, 0.0)},
}


def geometry(name):
    return RobotGeometry.load(properties=ARRANGEMENTS[name])


GEOMETRY = {name: geometry(name) for name in ARRANGEMENTS}


def test_each_arrangement_really_moves_its_sensor():
    base = GEOMETRY['measured']
    for name, frames in [('lidar_moved', [LIDAR]), ('camera_moved', [CAMERA]),
                         ('rail_moved', [LIDAR, CAMERA]), ('gps_moved', [GPS])]:
        for frame in frames:
            assert np.linalg.norm(GEOMETRY[name].pose(frame)[1] - base.pose(frame)[1]) > 0.1
            angle = math.acos((np.trace(GEOMETRY[name].pose(frame)[0].T @
                                        base.pose(frame)[0]) - 1) / 2)
            assert angle > 2 * DEG, (name, frame)


def test_the_measured_mounts_compose_from_the_datum():
    """The xacro's mount properties compose from the box-bottom datum (z datum_z in base_link)."""
    g, P = GEOMETRY['measured'], sensor_offsets.P
    z = P['datum_z']
    assert g.pose(LIDAR)[1] == pytest.approx(
        [P['lidar_x'], P['lidar_y'], z + P['lidar_z']], abs=1e-4)
    assert g.pose(GPS)[1] == pytest.approx([P['gps_x'], P['gps_y'], z + P['gps_z']], abs=1e-4)
    assert g.pose('imu_link')[1] == pytest.approx(
        [P['imu_x'], P['imu_y'], z + P['imu_z']], abs=1e-4)
    # ZED 2i: the lens is half the body depth ahead of the body centre, and the wrapper puts it
    # zed_lens_x behind and height/2 above its camera_link.
    lens = g.pose(CAMERA)[1] + [P['zed_lens_x'], 0.0, P['zed_height'] / 2]
    assert lens == pytest.approx(
        [P['camera_x'] + P['zed_depth'] / 2, P['camera_y'], z + P['camera_z']], abs=1e-4)


# ---------------------------------------------------------------- scene
BUOYS = [(-3.0, 9.0), (2.5, 7.0), (0.0, 12.0), (5.0, 15.0), (-6.0, 17.0)]   # (y, x) in base
RADIUS, HEIGHT = 0.25, 1.0          # pole buoys: vertical cylinders from the water up
NEAR = {i for i, (y, x) in enumerate(BUOYS) if math.hypot(x, y) < 13.0}
# Isolated, near buoys for the extrinsic check (it samples only those under 10 m).
SAMPLED = [(-2.5, 6.5), (0.0, 8.0), (2.5, 7.0)]


def buoys_base():
    return [np.array([x, y, WATER_Z]) for y, x in BUOYS]


def cylinder_surface(foot):
    a, h = np.meshgrid(np.linspace(0, 2 * math.pi, 48), np.linspace(0, HEIGHT, 12))
    return np.stack([foot[0] + RADIUS * np.cos(a.ravel()), foot[1] + RADIUS * np.sin(a.ravel()),
                     foot[2] + h.ravel()], 1)


def lidar_scan(g, feet):
    """VLP-16 hits (lidar_link coordinates) on the buoys, rays cast from the true mount."""
    R, o = g.pose(LIDAR)
    a, e = (v.ravel() for v in np.meshgrid(AZ, RINGS))
    d = (np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], 1)) @ R.T
    best = np.full(len(d), np.inf)
    for foot in feet:
        c = foot[:2] - o[:2]
        dh = d[:, :2]
        A = np.sum(dh * dh, 1)
        b = dh @ c
        disc = b * b - A * (c @ c - RADIUS ** 2)
        t = np.where(disc >= 0, (b - np.sqrt(np.maximum(disc, 0))) / np.maximum(A, 1e-12),
                     np.inf)
        z = o[2] + t * d[:, 2]
        hit = (t > 0) & (z >= foot[2]) & (z <= foot[2] + HEIGHT)
        best = np.where(hit & (t < best), t, best)
    ok = np.isfinite(best)
    return (d[ok] * best[ok, None]) @ R            # back into lidar axes


def optical(g):
    """Return base <- optical camera (R, t) for a description."""
    R, t = g.pose(CAMERA)
    return R @ lf.BODY_TO_OPTICAL.T, t


def observations(true, feet, stereo_scale=1.12):
    """Boxed stereo observations from the true camera, ranged 12% long (stereo bias)."""
    R, t = optical(true)
    fx, fy, cx, cy = K
    out = []
    for i, foot in enumerate(feet):
        P = (cylinder_surface(foot) - t) @ R
        if np.any(P[:, 2] <= 0.5):
            continue
        u, v = cx + fx * P[:, 0] / P[:, 2], cy + fy * P[:, 1] / P[:, 2]
        centre = (foot + [0, 0, HEIGHT / 2] - t) @ R * stereo_scale
        out.append(NS(class_id='red_pole_buoy', score=0.9, backend_id=str(i),
                      position=NS(x=centre[0], y=centre[1], z=centre[2]),
                      position_covariance=[0.0] * 9, has_box2d=True,
                      box2d=[u.min() / IMAGE[0], v.min() / IMAGE[1],
                             u.max() / IMAGE[0], v.max() / IMAGE[1]], truth=i))
    return out


def fuse(true, given, feet=None):
    """Run lidar_fusion.adjust on the true world through the given description."""
    feet = buoys_base() if feet is None else feet
    R_bl, t_bl = given.pose(LIDAR)
    R_bo, t_bo = optical(given)
    pts = (lidar_scan(true, feet) @ R_bl.T + t_bl - t_bo) @ R_bo       # optical axes, given camera
    obs = observations(true, feet)
    ctx = lf.Context(CFG, pm.TRACKER, K, IMAGE, np.eye(3))
    rows, _ = lf.adjust(obs, pts, ctx, 0.0)
    errors = {}
    for source, index, p, _, _, _ in rows:
        if source not in ('lidar', 'split'):
            continue
        centre = feet[obs[index].truth] + [0, 0, HEIGHT / 2]
        in_base = R_bo @ p + t_bo
        errors[obs[index].truth] = np.linalg.norm((in_base - centre)[:2])
    return errors, len(obs), (obs, pts, ctx, rows)


@pytest.mark.parametrize('name', ARRANGEMENTS)
def test_lidar_fusion_ranges_every_buoy_when_the_urdf_matches(name):
    errors, seen, _ = fuse(GEOMETRY[name], GEOMETRY[name])
    assert seen == len(BUOYS)
    # Every buoy within 13 m gets lidar (beyond, the stereo gate and ring count decide, as on
    # the boat); the 12% stereo bias (1-2 m) is gone, the near-face model leaves ~0.12 m.
    assert set(errors) == NEAR, errors
    assert max(errors.values()) < 0.15, errors


@pytest.mark.parametrize('name', ['lidar_moved', 'camera_moved', 'rail_moved'])
def test_a_moved_sensor_with_a_stale_urdf_breaks_fusion_or_is_caught(name):
    """The sensor moved, the URDF did not: either no box gets lidar or ranges go wrong."""
    errors, seen, _ = fuse(GEOMETRY[name], GEOMETRY['measured'])
    matched, _, _ = fuse(GEOMETRY[name], GEOMETRY[name])
    assert len(errors) < len(matched) or max(errors.values()) > 2 * max(matched.values())


def extrinsic_level(true, given, frames=80):
    """Feed the node's online lidar-camera check 24 s of a buoy approach; return the estimate."""
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    for k in range(frames):
        # The boat creeps up on the buoys, weaving: the field shifts a little every frame.
        shift = np.array([-0.03 * k, 0.3 * math.sin(0.5 * k), 0.0])
        feet = [np.array([x, y, WATER_Z]) + shift for y, x in SAMPLED]
        _, _, (obs, pts, ctx, rows) = fuse(true, given, feet)
        for ob in obs:
            ob.backend_id = f'{ob.backend_id}-{k // 10}'     # a fresh object every 3 s
        lf.sample_extrinsic(est, obs, pts, ctx, 0.3 * k, 0.0, rows)
    return est


@pytest.mark.parametrize('name', ['measured', 'rail_moved', 'lidar_moved', 'camera_moved'])
def test_the_online_extrinsic_check_is_quiet_when_the_urdf_matches(name):
    est = extrinsic_level(GEOMETRY[name], GEOMETRY[name])
    assert len(est.samples) >= CFG['extrinsic']['min_n']
    assert est.level == est.OK, est.message
    assert not est.alerts


@pytest.mark.parametrize('name', ['lidar_moved', 'camera_moved'])
def test_the_online_extrinsic_check_alarms_on_a_turned_sensor_the_urdf_missed(name):
    est = extrinsic_level(GEOMETRY[name], GEOMETRY['measured'])
    assert est.level != est.OK or est.alerts, (est.theta, est.se)


def test_a_whole_rail_move_is_invisible_to_the_lidar_camera_check():
    """Blind spot: a rail moved with both sensors on it keeps their relative extrinsic.

    The online check (and fusion_calib's du/dv) stays quiet while fused positions in base_link
    are off by the move. Only a check against something outside the rail sees it (a taped
    buoy's range and bearing in base_link, or the tracker against surveyed marks).
    """
    est = extrinsic_level(GEOMETRY['rail_moved'], GEOMETRY['measured'])
    assert est.level == est.OK and not est.alerts
    errors, _, _ = fuse(GEOMETRY['rail_moved'], GEOMETRY['measured'])
    assert min(errors.values()) > 0.4


# ---------------------------------------------------------------- tracker
def plant(t):
    """Circle of radius 4 m about the buoy field's near edge at 1.2 m/s."""
    w = 0.3
    yaw = w * t
    return sb.rz(yaw), np.array([4.0 * math.sin(yaw), 4.0 * (1 - math.cos(yaw)), 0.0])


WORLD_BUOYS = [(label, 11.0 * math.cos(a), 11.0 * math.sin(a)) for label, a in zip(
    ['red_buoy', 'green_buoy', 'yellow_buoy', 'black_buoy'] * 2,
    np.radians([0, 45, 90, 135, 180, 225, 270, 315]))]


def track(true, given, duration=25.0, rate=10.0):
    """World xy error of every confirmed static track against its buoy, at the end."""
    R_true, t_true = true.pose(CAMERA)
    R_given, t_given = given.pose(CAMERA)

    def pose_at(t, frame):
        R, p = plant(t)
        if frame == 'base_link':
            return R, p
        return R @ R_given, R @ t_given + p

    tracker = TrackFilter(pm.TRACKER)
    rng = np.random.default_rng(11)
    sim = dict(pm.SIM, phantoms={'names': []})
    for k in range(int(duration * rate)):
        t = 1.0 + k / rate
        pose = plant(t)
        dets = []
        for label, x, y in WORLD_BUOYS:
            p = sb.to_camera(np.array([x, y, WATER_Z]), pose, R_true, t_true)
            if sb.in_fov(p, R_true, sim['fov']):
                rho = float(np.linalg.norm(p))
                dets.append((label, 0.9, p + 0.02 * rho * rng.normal(size=3), np.ones(3) * .4))
        objects = sb.scene(dets, t, pose, R_true, t_true, sim, [])
        tracker.step(sb.observations(objects, round(t * 1e9), CAMERA, 'plain', R_true, K, IMAGE),
                     t + 0.1, pose_at)
    errors = []
    for tr in tracker.tracks:
        if tr.motion != MOTION_STATIC:
            continue
        d = [math.hypot(tr.x[0] - x, tr.x[1] - y) for label, x, y in WORLD_BUOYS
             if label == tr.class_id]
        errors.append(min(d))
    return errors


@pytest.mark.parametrize('name', ['measured', 'camera_moved', 'rail_moved'])
def test_tracks_land_on_the_buoys_when_the_urdf_matches(name):
    errors = track(GEOMETRY[name], GEOMETRY[name])
    assert len(errors) >= 6, errors
    assert max(errors) < 0.35, errors


def test_a_moved_camera_with_a_stale_urdf_puts_tracks_off_by_the_move():
    matched = track(GEOMETRY['camera_moved'], GEOMETRY['camera_moved'])
    stale = track(GEOMETRY['camera_moved'], GEOMETRY['measured'])
    # 0.3 m sideways and 5 deg of yaw: ~1 m at 11 m, so tracks miss or sit far off.
    assert len(stale) < len(matched) or np.median(stale) > 0.5, (stale, matched)


# ---------------------------------------------------------------- GNSS lever arm
def spin_fixes(true, roll_pitch=(0.0, 0.0)):
    """Antenna positions over a spin in place about base_link at the origin."""
    R_b, t_b = true.pose(GPS)
    out = []
    for yaw in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        R = rpy_matrix(roll_pitch[0], roll_pitch[1], yaw)
        out.append((R, R @ t_b))
    return out


def base_from_fix(fixes, given):
    """navsat_transform's lever arm: base = fix - R (base <- gps_link) from TF."""
    _, t = given.pose(GPS)
    return np.array([p - R @ t for R, p in fixes])


@pytest.mark.parametrize('name', ['measured', 'gps_moved'])
def test_the_lever_arm_cancels_in_a_spin_when_the_urdf_matches(name):
    for tilt in [(0.0, 0.0), (5 * DEG, -5 * DEG)]:
        base = base_from_fix(spin_fixes(GEOMETRY[name], tilt), GEOMETRY[name])
        assert np.ptp(base[:, 0]) < 1e-9 and np.ptp(base[:, 1]) < 1e-9


def test_a_stale_lever_arm_draws_its_circle_in_a_spin():
    """The spin's signature: base_link circles with radius |lever error| in xy."""
    base = base_from_fix(spin_fixes(GEOMETRY['gps_moved']), GEOMETRY['measured'])
    err = GEOMETRY['gps_moved'].pose(GPS)[1] - GEOMETRY['measured'].pose(GPS)[1]
    radius = np.hypot(*(base[:, :2] - base[:, :2].mean(0)).T)
    assert radius == pytest.approx(np.full(len(radius), np.hypot(*err[:2])), abs=1e-6)
