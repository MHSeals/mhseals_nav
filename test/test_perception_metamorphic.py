"""Metamorphic perception suite (perception-v2 §8): heading errors never make buoys move.

A deterministic plant (figure-eight, seed 7) feeds sim_backend's pure path (scene, inject,
observations; zed mode through zed_backend.convert_objects) into the real TrackFilter and
HeadingCheck. The camera noise is drawn identically in every case, so cases differ only by
the injected error. Relations R1-R5 are checked per backend mode over the case matrix.
"""
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from importlib.util import find_spec
from itertools import product
import math
from multiprocessing import get_context
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
try:
    import mhseals_nav.msg  # noqa: F401
except ImportError:  # host without the generated messages: duck types with the same fields
    def _vec():
        return SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class _Observation:
        VELOCITY_NONE, VELOCITY_RELATIVE, VELOCITY_WORLD = 0, 1, 2

        def __init__(self, class_id='', score=0.0):
            self.class_id, self.score, self.backend_id = class_id, score, ''
            self.position, self.size, self.velocity = _vec(), _vec(), _vec()
            self.position_covariance, self.velocity_covariance = [0.0] * 9, [0.0] * 9
            self.velocity_kind, self.has_box2d, self.box2d = 0, False, [0.0] * 4

    class _ObservationArray:
        def __init__(self):
            self.header = SimpleNamespace(frame_id='', stamp=SimpleNamespace(sec=0, nanosec=0))
            self.backend, self.processing_latency, self.observations = '', 0.0, []

    sys.modules['mhseals_nav.msg'] = SimpleNamespace(
        Observation=_Observation, ObservationArray=_ObservationArray)

from mhseals_nav import sim_backend as sb  # noqa: E402, I100
from mhseals_nav.heading_check import HeadingCheck, OK, WARN  # noqa: E402
from mhseals_nav.track_filter import MOTION_MOVING, MOTION_STATIC, TrackFilter  # noqa: E402

SEED, RATE, DURATION, T0 = 7, 15.0, 120.0, 1.0   # T0 keeps every TF lookup at t > 0
SPEED, YAW_RATE, RAMP = 1.5, 0.4, 3.0
HOLD = 12.775                 # s per lobe at full rate; closes the eight (solved numerically)
NOISE = (0.05, 0.02)          # sigma = a + b * range, per axis
DROPOUT = 0.05
MATCH_M = 3.0                 # track <-> truth; > range * (bias + extrinsic) at the 20 m edge
FRAME = 'front_camera_link'
R_BC, T_BC = np.eye(3), np.array([0.5, 0.0, 0.4])
BUOYS = [(label, 11.0 * math.cos(a), 11.0 * math.sin(a)) for label, a in zip(
    ['red_buoy', 'green_buoy', 'yellow_buoy', 'black_buoy'] * 2,
    np.radians([0, 45, 90, 135, 180, 225, 270, 315]))]
ELEMENTS = [{'label': label, 'x': x, 'y': y} for label, x, y in BUOYS]
BACKEND = yaml.safe_load((ROOT / 'config' / 'sim_backend.yaml').read_text())[
    '/**']['ros__parameters']
SIM = dict(BACKEND, phantoms={'names': ['boat'], 'boat': {
    'label': 'yellow_target_boat', 'centre': [0.0, 0.0], 'radius_m': 6.0, 'speed_mps': 0.8,
    'size': [1.0, 0.5, 0.4]}})
TRACKER = yaml.safe_load((ROOT / 'config' / 'tracker.yaml').read_text())[
    '/**']['ros__parameters']
IMAGE = (1280, 720)
K = sb.pinhole(IMAGE, SIM['fov']['hfov_deg'])


def case(bias=0.0, lag=0.0, ext=0.0, offset=0.0):
    return (('heading_bias_deg', bias), ('heading_lag_s', lag), ('extrinsic_yaw_deg', ext),
            ('stamp_offset_s', offset))


BASELINE = case()
SINGLES = [case(bias=5), case(bias=-5), case(lag=0.1), case(ext=3), case(ext=-3),
           case(offset=0.1), case(offset=-0.1)]
CORNERS = [case(b, 0.1, e, o) for b, e, o in product((-5, 5), (-3, 3), (-0.1, 0.1))]
ENVELOPE = [BASELINE] + SINGLES + CORNERS
LAGS = [case(lag=tau) for tau in (0.0, 0.1, 0.2, 0.3)]
# zed mode runs zed_backend, which needs ROS and lands with nav-camera: skipped per test
# until both are here, never for the whole module.
ZED = find_spec('rclpy') is not None and find_spec('mhseals_nav.zed_backend') is not None
needs_zed = pytest.mark.skipif(not ZED, reason='zed mode needs ROS and zed_backend')
MODES = ['plain', pytest.param('zed', marks=needs_zed)]


def yaw_rate(t):
    """+0.4 rad/s lobe, 3 s ramp, -0.4 rad/s lobe, 3 s ramp; one period returns to the start."""
    hold = HOLD
    phase = np.mod(np.asarray(t) - T0, 2 * (hold + RAMP))
    ramp = YAW_RATE * (1 - 2 * np.clip((phase - hold) / RAMP, 0, 1))
    back = -YAW_RATE * (1 - 2 * np.clip((phase - 2 * hold - RAMP) / RAMP, 0, 1))
    return np.where(phase < hold + RAMP, ramp, back)


class Plant:
    """Figure-eight truth at 1 kHz; pose_at interpolates, odom = world."""

    def __init__(self, dt=1e-3):
        self.t = np.arange(0.0, T0 + DURATION + 1.0, dt)
        self.r = yaw_rate(self.t)
        self.yaw = np.concatenate([[0.0], np.cumsum(self.r[:-1]) * dt])
        self.x = np.concatenate([[0.0], np.cumsum(SPEED * np.cos(self.yaw[:-1])) * dt])
        self.y = np.concatenate([[0.0], np.cumsum(SPEED * np.sin(self.yaw[:-1])) * dt])

    def base(self, t):
        if not self.t[0] <= t <= self.t[-1]:
            raise LookupError(t)
        yaw, x, y = (np.interp(t, self.t, a) for a in (self.yaw, self.x, self.y))
        return sb.rz(yaw), np.array([x, y, 0.0])

    def rate(self, t):
        return float(np.interp(t, self.t, self.r))

    def pose_at(self, t, frame):
        R, p = self.base(t)
        return (R, p) if frame == 'base_link' else (R @ R_BC, R @ T_BC + p)


PLANT = Plant()


def truth(t):
    """Labels and world xy of the buoys, then the phantom, at time t."""
    phantom = [(label, p[:2]) for label, p, _, _ in sb.phantoms(SIM['phantoms'], t)]
    labels = [label for label, _, _ in BUOYS] + [label for label, _ in phantom]
    return labels, np.array([(x, y) for _, x, y in BUOYS] + [xy for _, xy in phantom])


def score(track, labels, xy):
    """Index of the same-class truth object within MATCH_M of the track, else None."""
    d = np.hypot(*(xy - track.x[:2]).T)
    d[[label != track.class_id for label in labels]] = np.inf
    i = int(np.argmin(d))
    return i if d[i] <= MATCH_M else None


NO_TANGENTIAL = (('heading_sigma_deg', 0.0), ('lag_sigma_s', 0.0))
WORST = case(5, 0.1, 3, -0.1)   # the lag and stamp offset add up to 0.2 s


@lru_cache(maxsize=None)
def sessions():
    """Every session the tests read, simulated in parallel (each one is deterministic)."""
    modes = ['plain'] + (['zed'] if ZED else [])
    jobs = [(m, c, ()) for m in modes for c in ENVELOPE + LAGS[2:]]
    jobs.append(('plain', WORST, NO_TANGENTIAL))
    # spawn, not fork: rclpy (imported by the ROS pytest plugins) already runs threads.
    with ProcessPoolExecutor(mp_context=get_context('spawn')) as pool:
        return dict(zip(jobs, pool.map(simulate, *zip(*jobs))))


def run(mode, injection, tracker_meas=()):
    return sessions()[mode, injection, tracker_meas]


def simulate(mode, injection, tracker_meas):
    """One 120 s session; returns per-frame scored tracks and the final heading result."""
    inj = dict(injection)
    cfg = dict(TRACKER, meas=dict(TRACKER['meas'], **dict(tracker_meas)))
    tracker, check = TrackFilter(cfg), HeadingCheck(cfg['heading_check'])
    rng = np.random.default_rng(SEED)
    frames = []
    for k in range(int(DURATION * RATE)):
        t = T0 + k / RATE
        pose = PLANT.base(t)
        noise = rng.normal(size=(len(BUOYS), 3))   # drawn for every buoy in every frame
        kept = rng.random(len(BUOYS)) >= DROPOUT
        dets = []
        for (label, x, y), n, keep in zip(BUOYS, noise, kept):
            p = sb.to_camera(np.array([x, y, 0.0]), pose, R_BC, T_BC)
            if keep and sb.in_fov(p, R_BC, SIM['fov']):
                rho = float(np.linalg.norm(p))
                dets.append((label, 0.9, p + (NOISE[0] + NOISE[1] * rho) * n, np.ones(3) * .4))
        objects = sb.scene(dets, t, pose, R_BC, T_BC, SIM, ELEMENTS)
        objects = sb.inject(objects, R_BC, T_BC, sb.heading_error(inj, PLANT.rate(t)),
                            math.radians(inj['extrinsic_yaw_deg']))
        stamp = round((t + inj['stamp_offset_s']) * 1e9)
        tracker.step(sb.observations(objects, stamp, FRAME, mode, R_BC, K, IMAGE),
                     t + 0.1, PLANT.pose_at)
        for sample in tracker.heading_samples:
            check.add(*sample)
        labels, xy = truth(t)
        frames.append((t, [(tr.id, tr.class_id, tr.motion, score(tr, labels, xy))
                           for tr in tracker.tracks]))
    return SimpleNamespace(frames=frames, heading=check.result(T0 + DURATION))


def static_set(result, window=10.0):
    """(class, buoy) pairs held by a confirmed STATIC track during the last `window` s."""
    end = result.frames[-1][0]
    return {(c, m) for t, tracks in result.frames if t > end - window
            for _, c, motion, m in tracks
            if motion == MOTION_STATIC and m is not None and m < len(BUOYS)}


def moving_buoys(result):
    return sorted({(round(t, 2), c, m) for t, tracks in result.frames
                   for _, c, motion, m in tracks
                   if motion == MOTION_MOVING and m is not None and m < len(BUOYS)})


def id_switches(result):
    """Per truth object: how often its oldest scored track changes id between frames."""
    last, switches = {}, {}
    for _, tracks in result.frames:
        oldest = {}
        for ident, _, _, m in tracks:
            if m is not None:
                oldest[m] = min(ident, oldest.get(m, ident))
        for m, ident in oldest.items():
            switches[m] = switches.get(m, 0) + (m in last and last[m] != ident)
            last[m] = ident
    return switches


def test_plant_is_a_figure_eight_inside_the_buoy_ring():
    assert float(np.max(np.hypot(PLANT.x, PLANT.y))) < 11.0 - 2.0
    period = 2 * (HOLD + RAMP)
    assert np.hypot(*(PLANT.base(T0 + period)[1] - PLANT.base(T0)[1])[:2]) < 0.05
    assert PLANT.r.max() == pytest.approx(YAW_RATE) and PLANT.r.min() == pytest.approx(-YAW_RATE)


def test_plain_baseline_lifts_every_buoy_onto_its_truth():
    """Without noise or injection the observation lifted through pose_at is the truth."""
    t = T0 + 7.3
    pose = PLANT.base(t)
    dets = [(label, 0.9, sb.to_camera(np.array([x, y, 0.0]), pose, R_BC, T_BC), np.ones(3))
            for label, x, y in BUOYS]
    array = sb.observations(sb.scene(dets, t, pose, R_BC, T_BC, SIM, ELEMENTS),
                            round(t * 1e9), FRAME, 'plain', R_BC, K, IMAGE)
    R, p = PLANT.pose_at(t, FRAME)
    for ob, (_, x, y) in zip(array.observations, BUOYS):
        lifted = R @ [ob.position.x, ob.position.y, ob.position.z] + p
        assert lifted[:2] == pytest.approx([x, y], abs=1e-9)


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('injection', ENVELOPE, ids=lambda c: '-'.join(f'{v:g}' for _, v in c))
def test_r1_r2_static_buoys_stay_static_and_the_same(mode, injection):
    result = run(mode, injection)
    assert moving_buoys(result) == [], 'R1: a static buoy was classified moving'
    assert static_set(result) == static_set(run(mode, BASELINE)), 'R2: static set changed'


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('injection', ENVELOPE + LAGS[2:],
                         ids=lambda c: '-'.join(f'{v:g}' for _, v in c))
def test_r3_the_phantom_boat_is_moving_soon_after_confirmation(mode, injection):
    phantom = len(BUOYS)
    seen = [(t, motion) for t, tracks in run(mode, injection).frames
            for _, _, motion, m in tracks if m == phantom]
    assert seen, 'R3: the phantom was never confirmed'
    assert all(motion == MOTION_MOVING for t, motion in seen if t >= seen[0][0] + 2.0), \
        'R3: the phantom was not MOVING 2 s after confirmation'


@pytest.mark.parametrize('mode', MODES)
def test_r4_heading_check_recovers_the_injected_lag(mode):
    results = [run(mode, c).heading for c in LAGS]
    assert results[0]['level'] == OK and results[0]['excited']
    assert results[-1]['level'] >= WARN
    lags = [r['lag_s'] for r in results]
    assert lags == sorted(lags), f'R4: lag_s not monotone in tau: {lags}'
    assert abs(lags[-1] - 0.3) <= 0.07, f'R4: lag_s {lags[-1]:.3f} at tau 0.3'


@needs_zed
@pytest.mark.parametrize('injection', ENVELOPE, ids=lambda c: '-'.join(f'{v:g}' for _, v in c))
def test_r5_zed_ids_switch_no_more_than_plain(injection):
    zed, plain = id_switches(run('zed', injection)), id_switches(run('plain', injection))
    worse = {m: (n, plain.get(m, 0)) for m, n in zed.items() if n > plain.get(m, 0)}
    assert not worse, f'R5: more id switches with zed ids (zed, plain): {worse}'


def test_mutation_without_the_tangential_term_breaks_r1():
    """The suite must notice if R_pos loses its heading/lag term."""
    mutant = run('plain', WORST, NO_TANGENTIAL)
    assert moving_buoys(mutant) or static_set(mutant) != static_set(run('plain', BASELINE))
