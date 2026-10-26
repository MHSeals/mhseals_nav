"""lidar_fusion pure parts against synthetic scans (design lidar-fusion-design.md rev 2)."""
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS

from mhseals_nav import lidar_fusion as lf
from mhseals_nav.track_filter import TrackFilter
import numpy as np
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
CFG = yaml.safe_load((CONFIG / 'lidar_fusion.yaml').read_text())['/**']['ros__parameters']
TRACKER = yaml.safe_load((CONFIG / 'tracker.yaml').read_text())['/**']['ros__parameters']
K = (640.0, 640.0, 640.0, 360.0)
IMAGE = (1280, 720)
DEG = math.pi / 180
RINGS = np.radians(np.arange(-15, 16, 2))
AZ = np.radians(np.arange(-60, 60, 0.2))


def ctx(**kw):
    return lf.Context(CFG, TRACKER, K, IMAGE, np.eye(3), **kw)


def scan(objects, rings=RINGS, az=AZ):
    """Optical-axes hits of rays from the origin on vertical cylinders (X, Z, R, y_top, y_bot).

    Y is down: a buoy spans y_top (its top, negative) to y_bot (the water line).
    """
    a, e = np.meshgrid(az, rings)
    a, e = a.ravel(), e.ravel()
    dh = np.stack([np.sin(a), np.cos(a)], 1)
    best = np.full(len(a), np.inf)
    for X, Z, R, y_top, y_bot in objects:
        c = np.array([X, Z])
        b = dh @ c
        disc = b ** 2 - (c @ c - R ** 2)
        t = np.where(disc >= 0, b - np.sqrt(np.maximum(disc, 0)), np.inf)
        y = -np.tan(e) * t
        hit = (t > 0) & (y >= y_top) & (y <= y_bot)
        best = np.where(hit & (t < best), t, best)
    ok = np.isfinite(best)
    t = best[ok]
    return np.stack([np.sin(a[ok]) * t, -np.tan(e[ok]) * t, np.cos(a[ok]) * t], 1)


def box_of(X, Z, R, y_top, y_bot, pad=0.0):
    fx, fy, cx, cy = K
    W, H = IMAGE
    az = math.atan2(X, Z)
    rho = math.hypot(X, Z)
    half = math.asin(min(1.0, R / rho))
    u0, u1 = cx + fx * math.tan(az - half) - pad, cx + fx * math.tan(az + half) + pad
    near = rho - R
    v0, v1 = cy + fy * y_top / near, cy + fy * y_bot / near
    return [u0 / W, v0 / H, u1 / W, v1 / H]


def observation(X, Z, R=0.2, y_top=0.4, y_bot=1.0, rho_scale=1.0, cls='red_buoy', bid='7',
                box=None, cov=None):
    """Return a stereo observation at the cylinder's centre, its range scaled by rho_scale."""
    rho = math.hypot(X, Z)
    centre = np.array([X, (y_top + y_bot) / 2, Z]) * rho_scale
    return NS(class_id=cls, score=0.9, backend_id=bid,
              position=NS(x=centre[0], y=centre[1], z=centre[2]),
              position_covariance=[0.0] * 9 if cov is None else cov,
              has_box2d=True, box2d=box or box_of(X, Z, R, y_top, y_bot), rho=rho)


def lidar_rows(rows):
    return [r for r in rows if r[0] in ('lidar', 'split')]


# -------------------------------------------------------------- geometry
def test_rpy_round_trip_and_small_rotation_match():
    R = lf.rpy_matrix(0.1, -0.2, 2.5)
    assert np.allclose(lf.matrix_rpy(R), (0.1, -0.2, 2.5))
    w = np.array([0.01, -0.02, 0.005])
    assert np.allclose(lf.small_rotation(w) @ lf.small_rotation(-w), np.eye(3))


def test_compensation_returns_a_static_point_to_its_place_at_t_c_during_a_spin():
    """Base turning at 1.5 rad/s and moving at 2 m/s: points seen over 0.1 s land on t_c."""
    def pose(t):
        return lf.rpy_matrix(0, 0, 1.5 * t), np.array([2.0 * t, 0.3 * t, 0.0])
    world = np.array([10.0, 2.0, -0.5])
    t_i = np.linspace(-0.1, 0.0, 50)
    seen = np.array([pose(t)[0].T @ (world - pose(t)[1]) for t in t_i])
    t_c = -0.03
    knot_t = np.append(np.linspace(t_i.min(), t_i.max(), 5), t_c)
    knot_t.sort()
    out = lf.compensate(seen, t_i, knot_t, lf.delta_knots([pose(t) for t in knot_t], pose(t_c)))
    expect = pose(t_c)[0].T @ (world - pose(t_c)[1])
    assert np.abs(out - expect).max() < 2e-3


def test_cloud_parsing_keeps_finite_points_beyond_r_min_with_their_time():
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4'), ('time', '<f4')])
    data = np.array([(5, 0, 0, 1, -0.05), (1, 0, 0, 1, -0.01), (np.nan, 0, 0, 1, 0),
                     (0, 3, 1, 1, -0.09)], dtype)
    fields = [NS(name=n, offset=dtype.fields[n][1], datatype=7) for n in dtype.names]
    msg = NS(fields=fields, width=4, height=1, is_bigendian=False, point_step=20,
             data=data.tobytes())
    xyz, tau = lf.cloud_arrays(msg, 2.0)
    assert xyz.tolist() == [[5, 0, 0], [0, 3, 1]]
    assert tau == pytest.approx([-0.05, -0.09])
    msg.fields = fields[:3]
    assert lf.cloud_arrays(msg, 2.0)[1].tolist() == [0.0, 0.0]


def test_scan_choice_and_tf_gap():
    scans = [{'t_med': 1.0}, {'t_med': 1.1}, {'t_med': 1.2}]
    assert lf.choose_scan(scans, 1.13, 0.1)['t_med'] == 1.1
    assert lf.choose_scan(scans, 1.35, 0.1) is None
    stamps = [0.0, 0.01, 0.02, 0.1, 0.11]
    assert lf.tf_gap(stamps, 0.005, 0.015) == pytest.approx(0.01)
    assert lf.tf_gap(stamps, 0.015, 0.105) == pytest.approx(0.08)
    assert lf.tf_gap(stamps, 0.05, 0.2) == math.inf


def test_crop_keeps_the_camera_sector():
    pts = np.array([[10, 0, 0], [0, 10, 0], [-10, 0, 0], [10, 9, 0]], float)
    keep = lf.crop_mask(pts, np.zeros(3), np.array([1.0, 0, 0]), math.radians(60))
    assert keep.tolist() == [True, False, False, True]


# ------------------------------------------------------------ clusters
def test_clusters_split_side_by_side_buoys_and_in_line_ones():
    P = scan([(-0.5, 8, 0.2, 0.4, 1.0), (0.5, 8, 0.2, 0.4, 1.0), (0.0, 12, 0.2, 0.4, 1.0)])
    rho = np.hypot(P[:, 0], P[:, 2])
    cls = lf.clusters(rho, np.arctan2(P[:, 0], P[:, 2]), CFG)
    assert len(cls) == 3


# ------------------------------------------------------------ adjustment
def test_lidar_reranges_a_biased_stereo_buoy_along_its_own_ray():
    X, Z = 1.5, 9.0
    P = scan([(X, Z, 0.2, 0.4, 1.0)])
    ob = observation(X, Z, rho_scale=1.15)
    rows, _ = lf.adjust([ob], P, ctx(), 0.0)
    (source, _, p, cov, _, bid), = rows
    assert source == 'lidar' and bid == '7'
    s = np.array([ob.position.x, ob.position.y, ob.position.z])
    assert np.allclose(p / np.linalg.norm(p), s / np.linalg.norm(s))
    assert abs(math.hypot(p[0], p[2]) - ob.rho) < 0.13 * 0.2 + 0.02


@pytest.mark.parametrize('R,shape', [(0.25, 'pole'), (0.2, 'ball')])
def test_near_face_offset_recovers_the_centre_within_the_known_bias(R, shape):
    errs = []
    cylinder = lf.Context({**CFG, 'near_face_k': {'default': 0.87}}, TRACKER, K, IMAGE,
                          np.eye(3))     # the synthetic bodies are vertical cylinders
    for rho in (3, 6, 10, 15):
        P = scan([(0.0, rho, R, 0.3 if shape == 'ball' else -0.5, 1.0)])
        cls = 'red_pole_buoy' if shape == 'pole' else 'red_buoy'
        rows, _ = lf.adjust([observation(0.0, rho, R, 0.3 if shape == 'ball' else -0.5,
                                         cls=cls, rho_scale=1.05)], P, cylinder, 0.0)
        (source, _, p, *_), = rows
        assert source == 'lidar'
        errs.append(math.hypot(p[0], p[2]) - rho)
    assert max(abs(e) for e in errs) < 0.13 * R + 0.01


def test_near_face_k_takes_the_longest_class_suffix():
    """Pole buoys, balls and the rest each get their own near-face k."""
    c = lf.Context({**CFG, 'near_face_k': {'default': 0.9, 'buoy': 0.8, 'pole_buoy': 0.4}},
                   TRACKER, K, IMAGE, np.eye(3))
    assert [c.near_face(n) for n in ('red_pole_buoy', 'green_buoy', 'black_cross')] == \
        [0.4, 0.8, 0.9]


def test_lidar_covariance_is_bias_aware_and_stereo_matches_the_tracker_total():
    rates = lf.RateEma(CFG['rate_alpha'], CFG['default_hz'])
    for k in range(30):
        rates.update('7', k / 15)
    X, Z = 0.0, 8.0
    P = scan([(X, Z, 0.2, 0.4, 1.0)])
    rows, _ = lf.adjust([observation(X, Z)], P, ctx(), 2.0, rates=rates)
    cov = np.array(rows[0][3]).reshape(3, 3)
    f = rates.hz['7']
    sb2 = (CFG['bias_radius_frac'] * 0.2) ** 2 + CFG['sigma_t_ext_m'] ** 2
    expect = CFG['sigma_w_m'] ** 2 + f * sb2
    assert cov[2, 2] == pytest.approx(expect, rel=1e-6)
    assert f == pytest.approx(15, rel=0.05)
    # Stereo pass: the fused tracker (depth_term false) sees the radial total of today's.
    ob = observation(X, Z, rho_scale=1.2)
    (_, _, p, cov, *_), = lf.adjust([ob], np.zeros((0, 3)), ctx(), 0.0)[0]
    fused = copy.deepcopy(TRACKER)
    fused['meas']['depth_term'] = False
    # Tracker axes: camera looking along +x (body): optical (X, Y, Z) -> body (Z, -X, -Y).
    B = lf.BODY_TO_OPTICAL.T
    body = NS(**vars(ob))
    body.position = NS(**dict(zip('xyz', B @ p)))
    body.position_covariance = (B @ np.array(cov).reshape(3, 3) @ B.T).ravel().tolist()
    body.velocity_kind = 0
    plain = NS(**vars(body))
    plain.position_covariance = [0.0] * 9
    args = (np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    _, Rz_fused, _ = TrackFilter(fused)._measurement(body, *args)
    _, Rz_today, _ = TrackFilter(TRACKER)._measurement(plain, *args)
    assert Rz_fused[0, 0] == pytest.approx(Rz_today[0, 0], rel=1e-9)


def test_box_less_observation_keeps_the_stereo_model_and_nan_passes():
    ob = observation(1.0, 6.0)
    ob.has_box2d = False
    bad = observation(1.0, 6.0)
    bad.position.x = math.nan
    rows, _ = lf.adjust([ob, bad], scan([(1.0, 6.0, 0.2, 0.4, 1.0)]), ctx(), 0.0)
    assert [r[0] for r in rows] == ['stereo', 'pass']
    cov = np.array(rows[0][3]).reshape(3, 3)
    rho = math.hypot(ob.position.x, ob.position.z)
    floor = max(TRACKER['meas']['min_sigma_m'], TRACKER['meas']['range_frac'] * rho) ** 2
    u = np.array([ob.position.x, 0, ob.position.z]) / rho
    n = np.cross(lf.UP, u)
    assert u @ cov @ u == pytest.approx(floor + (TRACKER['meas']['depth_k'] * rho ** 2) ** 2)
    assert n @ cov @ n == pytest.approx(floor)
    assert rows[1][2] is None


def test_spray_and_sparse_points_never_range_a_box():
    rng = np.random.default_rng(3)
    spray = np.column_stack([rng.uniform(-1, 1, 6), rng.uniform(-0.2, 0.8, 6),
                             rng.uniform(7.5, 8.5, 6)])
    rows, _ = lf.adjust([observation(0.0, 8.0)], spray, ctx(), 0.0)
    assert rows[0][0] == 'stereo'


def test_in_line_buoys_inside_the_stereo_gate_are_ambiguous_but_separate_when_far():
    near = [(0.0, 8.0, 0.2, 0.4, 1.0), (0.0, 8.6, 0.2, -0.3, 1.0)]   # a taller ball behind
    rows, _ = lf.adjust([observation(0.0, 8.0, rho_scale=1.06)], scan(near), ctx(), 0.0)
    assert rows[0][0] == 'stereo'        # rule (a): two feasible clusters at similar cost
    pole = [(0.0, 8.0, 0.2, 0.4, 1.0), (0.0, 8.6, 0.2, -1.5, 1.0)]   # a pole over the ball
    rows, _ = lf.adjust([observation(0.0, 8.0, rho_scale=1.06)], scan(pole), ctx(), 0.0)
    assert rows[0][0] == 'lidar'         # the pole spans more rings than the ball's box
    assert math.hypot(rows[0][2][0], rows[0][2][2]) == pytest.approx(8.0, abs=0.1)
    far = scan([(0.0, 4.0, 0.2, 0.4, 1.0), (0.0, 12.0, 0.2, -0.5, 1.0)])
    rows, _ = lf.adjust([observation(0.0, 4.0)], far, ctx(), 0.0)
    assert rows[0][0] == 'lidar'


def test_one_cluster_in_two_overlapping_boxes_is_contested():
    X, Z = 0.0, 8.0
    P = scan([(X, Z, 0.2, 0.4, 1.0)])
    a = observation(X, Z, bid='1')
    rows, _ = lf.adjust([a, observation(X + 0.03, Z, bid='2')], P, ctx(), 0.0)
    assert [r[0] for r in rows] == ['stereo', 'stereo']      # rule (b): costs within 0.15
    rows, _ = lf.adjust([a, observation(X + 0.15, Z, bid='2')], P, ctx(), 0.0)
    assert [r[0] for r in rows] == ['lidar', 'stereo']       # a clear winner keeps it


@pytest.mark.parametrize('gap', [0.6, 0.7, 1.0, 1.5])
def test_a_merged_box_splits_into_one_observation_per_buoy(gap):
    """A merged pair splits beyond the ~0.93 m that whole-box IoU >= 0.3 allowed."""
    Z = 8.0
    objs = [(-gap / 2, Z, 0.2, 0.4, 1.0), (gap / 2, Z + 0.2, 0.2, 0.4, 1.0)]
    P = scan(objs)
    b0, b1 = box_of(*objs[0]), box_of(*objs[1])
    union = [b0[0], min(b0[1], b1[1]), b1[2], max(b0[3], b1[3])]
    ob = observation(gap / 2 * 0.9, Z * 1.1, box=union)   # stereo bearing nearer the far one
    rows, _ = lf.adjust([ob], P, ctx(), 0.0)
    assert [r[0] for r in rows] == ['split', 'split']
    xs = sorted(r[2][0] for r in rows)
    assert xs == pytest.approx([-gap / 2, gap / 2], abs=0.08)
    assert sorted(r[5] for r in rows) == ['', '7']
    assert [r[5] for r in sorted(rows, key=lambda r: r[2][0])][0] == '7'   # the nearer keeps it
    # The keyed part's covariance covers the jump from the union (where its track sat).
    keyed = next(r for r in rows if r[5] == '7')
    d = keyed[2] - np.array([ob.position.x, ob.position.y, ob.position.z])
    assert d @ np.linalg.solve(np.reshape(keyed[3], (3, 3)), d) < 1.0


def test_a_wall_behind_a_far_ball_no_ring_hits_is_too_wide():
    """Only a wall 3 m behind returns inside the ball's window: the full extent is wide."""
    wall = [(x, 21.0, 0.2, -0.5, 1.0) for x in np.arange(-3.0, 3.01, 0.3)]
    P = scan(wall)
    rows, _ = lf.adjust([observation(0.0, 18.0, y_top=0.75, rho_scale=1.05)], P, ctx(), 0.0)
    assert rows[0][0] == 'stereo'
    lone = scan([(0.0, 21.0, 0.2, -0.5, 1.0)])          # a single post there still ranges
    rows, _ = lf.adjust([observation(0.0, 18.0, y_top=-0.5, rho_scale=1.05)], lone, ctx(), 0.0)
    assert rows[0][0] == 'lidar'


def test_a_boat_gives_no_samples_and_does_not_count_as_silent():
    obs = [observation(0.0, 6.0, R=0.6, cls='black_target_boat')]
    assert lf.extrinsic_samples(obs, scan([(0.0, 6.0, 0.6, 0.4, 1.0)]), ctx()) == []


def test_held_ratio_decays_to_stereo_as_an_ar1_bias_for_two_seconds():
    held = lf.HeldRatio(CFG['held_alpha'], CFG['held_s'], CFG['held_tau_s'])
    X, Z = 0.0, 8.0
    P = scan([(X, Z, 0.2, 0.4, 1.0)])
    for k in range(10):
        lf.adjust([observation(X, Z, rho_scale=1.2)], P, ctx(), k * 0.1, held=held)
    sig = []
    for t in (0.95, 1.9, 2.95):
        frame = [observation(X, Z, rho_scale=1.2)]
        (source, _, p, cov, *_), = lf.adjust(frame, np.zeros((0, 3)), ctx(), t, held=held)[0]
        sig.append(np.array(cov).reshape(3, 3)[2, 2])
        if t < 2.9:
            assert source == 'held'
            phi = math.exp(-(t - 0.9) / CFG['held_tau_s'])
            assert math.hypot(p[0], p[2]) == pytest.approx(1.2 * Z - 0.2 * Z * phi, abs=0.1)
    assert source == 'stereo'
    assert sig[0] < sig[1] < (TRACKER['meas']['depth_k'] * 1.2 ** 2 * Z ** 2) ** 2 + sig[0]


# ------------------------------------------------------------ extrinsic
def fake_samples(est, w, c=(0, 0, 0), tau=0.0, n=60, seed=1, noise=True, h=12.0, keyed=True):
    rng = np.random.default_rng(seed)
    fx, fy, cx, cy = K
    for k in range(n):
        x, y = rng.uniform(-1.0, 1.0), rng.uniform(0.02, 0.15)
        Z = rng.uniform(2.0, 10.0)
        yaw_rate = rng.uniform(-1, 1)
        rows = lf.model_rows(x, y, Z, yaw_rate)
        theta = np.array([*w, *c, tau])
        r_az = float(np.dot(rows[0], theta)) + noise * rng.normal(0, 1.0 / fx)
        r_el = float(np.dot(rows[1], theta)) + noise * rng.normal(0, 3e-3)
        b = NS(u0=cx + fx * x - 5, u1=cx + fx * x + 5, v0=cy + fy * y - h / 2,
               v1=cy + fy * y + h / 2)
        cl = NS(uc=cx + fx * math.tan(math.atan(x) + r_az),
                vc=cy + fy * math.tan(math.atan(y) + r_el), rho=Z)
        est.add(k * 0.25, str(k) if keyed else '', b, cl, ctx(), yaw_rate)


def test_key_less_samples_stay_separate_objects_once_the_window_is_full():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, (0, 0, 0), n=2 * CFG['extrinsic']['window_n'], keyed=False)
    keys = [s[3] for s in est.samples]
    assert len(est.samples) == CFG['extrinsic']['window_n'] and len(set(keys)) == len(keys)


def test_extrinsic_fit_recovers_rotation_translation_and_time():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    w = np.radians([0.4, 0.6, -0.5])
    fake_samples(est, w, c=(0.03, -0.02, 0.0), tau=0.01, noise=False)
    theta, _ = est.fit()
    assert np.degrees(theta[:3]) == pytest.approx(np.degrees(w), abs=0.05)
    assert theta[3:5] == pytest.approx([0.03, -0.02], abs=0.01)
    assert theta[6] == pytest.approx(0.01, abs=0.002)


def test_off_centre_offset_does_not_leak_into_yaw():
    """A 5 cm lateral offset seen at 45 deg and 2 m: the model's (1+x^2) keeps yaw clean."""
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, (0, 0, 0), c=(0.05, 0, 0))
    theta, se = est.fit()
    assert abs(math.degrees(theta[1])) < max(0.1, 4 * math.degrees(se[1]))
    assert theta[3] == pytest.approx(0.05, abs=0.02)


def test_one_degree_warns_and_applies_at_most_one_degree():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, np.radians([0, 1.2, 0]), n=200)
    assert est.level == est.WARN
    assert math.degrees(est.applied[1]) == pytest.approx(1.0)


def test_half_degree_is_corrected_without_alarm():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, np.radians([0, 0.5, 0]), n=200)
    assert est.level == est.OK
    assert math.degrees(est.applied[1]) == pytest.approx(0.5, abs=0.1)


def test_each_axis_applies_only_on_its_own_se():
    """A weak pitch fit is not applied while a precise yaw fit is."""
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, np.radians([0.5, 0.5, 0]), n=60, h=80.0)   # tall boxes: weak r_el
    assert math.degrees(est.se[0]) > CFG['extrinsic']['apply_se_deg']
    assert est.applied[0] == 0.0
    assert math.degrees(est.applied[1]) == pytest.approx(0.5, abs=0.1)


def three_buoys(az_deg, el_deg):
    """60 samples of three buoys (30 px boxes) with their own (r_az, r_el) biases."""
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fx, fy, cx, cy = K
    rng = np.random.default_rng(3)
    for k in range(60):
        x, y, Z = rng.uniform(-0.8, 0.8), rng.uniform(0.02, 0.15), rng.uniform(3.0, 9.0)
        r_az, r_el = math.radians(az_deg[k % 3]), math.radians(el_deg[k % 3] + rng.normal(0, 0.3))
        b = NS(u0=cx + fx * x - 5, u1=cx + fx * x + 5, v0=cy + fy * y - 15, v1=cy + fy * y + 15)
        cl = NS(uc=cx + fx * math.tan(math.atan(x) + r_az),
                vc=cy + fy * math.tan(math.atan(y) + r_el), rho=Z)
        est.add(k * 0.25, f'buoy{k % 3}', b, cl, ctx(), 0.0)
    return est


def test_a_few_buoys_own_biases_raise_no_alarm_but_3_deg_pitch_does():
    """Per-buoy biases at the sim E0 worst spread (S6) stay quiet.

    3 deg of pitch on the same three buoys raises ERROR: scaling the SE by
    sqrt(N / objects) hid it (S4).
    """
    assert three_buoys((0.5, -0.3, 0.4), (0.5, -0.4, 0.2)).level == lf.ExtrinsicEstimate.OK
    assert three_buoys((0, 0, 0), (-3, -3, -3)).level == lf.ExtrinsicEstimate.ERROR


def test_two_degrees_raise_error_and_stop_applying():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, np.radians([0, 0, 2.0]), n=60)
    assert est.level == est.ERROR and not np.any(est.applied)
    fake_samples(est, (0, 0, 0), n=200, seed=4)
    assert est.level == est.ERROR             # latched until restart


def test_translation_and_time_alert_but_never_apply():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    fake_samples(est, (0, 0, 0), c=(0.1, 0.0, 0.0), tau=0.03, n=200)
    assert {'translation', 'image_time'} <= est.alerts
    assert est.level == est.OK and np.abs(est.applied).max() < math.radians(0.3)


def test_no_samples_over_many_isolated_boxes_is_a_silent_failure():
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    for k in range(60):
        est.observe_isolated(k * 0.4, False, False)
    assert est.level == est.ERROR
    ok = lf.ExtrinsicEstimate(CFG['extrinsic'])
    for k in range(60):
        ok.observe_isolated(k * 0.4, k % 5 == 0, True)
    assert ok.level == ok.OK


def test_samples_from_a_rotated_scan_measure_the_rotation():
    """End to end: lidar turned 0.8 deg in yaw, isolated buoys across the image."""
    w = np.array([0.0, math.radians(0.8), 0.0])
    est = lf.ExtrinsicEstimate(CFG['extrinsic'])
    rng = np.random.default_rng(5)
    t = 0.0
    for k in range(60):
        objs = [(rng.uniform(-4, 4), rng.uniform(4, 9.5), 0.25, -0.5, 1.0)]
        P = scan(objs) @ lf.small_rotation(w).T
        for b, c in lf.extrinsic_samples([observation(*objs[0][:2], R=0.25, y_top=-0.5,
                                                      cls='red_pole_buoy', bid=str(k))],
                                         P, ctx()):
            assert c is not None
            est.add(t, b.ob.backend_id, b, c, ctx(), 0.0)
        t += 0.3
    theta, se = est.fit()
    assert math.degrees(theta[1]) == pytest.approx(0.8, abs=0.15)


@pytest.mark.parametrize('shape', [(1, 1), (3, 3), (2, 5), (5, 2), (4, 6)])
def test_assignment_is_the_brute_force_optimum(shape):
    from itertools import permutations
    rng = np.random.default_rng(sum(shape))
    for _ in range(20):
        C = rng.uniform(0, 1, shape)
        C[rng.uniform(size=shape) < 0.3] = 1e6
        pairs = lf.assignment(C)
        n = min(shape)
        assert len(pairs) == n and len({r for r, _ in pairs}) == len({c for _, c in pairs}) == n
        if shape[0] <= shape[1]:
            best = min(sum(C[i, p[i]] for i in range(n)) for p in permutations(range(shape[1]), n))
        else:
            best = min(sum(C[p[j], j] for j in range(n)) for p in permutations(range(shape[0]), n))
        assert sum(C[r, c] for r, c in pairs) == pytest.approx(best)
