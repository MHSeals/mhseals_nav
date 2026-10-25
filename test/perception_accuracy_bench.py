"""Perception accuracy bench: before/after tracker and waterline settings on one plant.

Not a pytest module (run it: ``python3 test/perception_accuracy_bench.py [--quick]``); the
guard tests in test_waterline.py run a short version.

Plant (seeded): the boat weaves at 1.5 m/s with yaw-rate steps of up to +-0.6 rad/s and 0.8 s
ramps through a ring field of buoys at 6-24 m, rolling about 4 deg and pitching about 2.5 deg
(two tones each), heaving 3 cm. TF has no roll or pitch (ekf two_d_mode) and its heading is
the truth sampled at the IMU rate, held, and delayed; TF reaches only a short way past each
capture stamp when the frame is processed, as on the boat.

Camera (ZED 2i stereo, as fitted on 2026-10-25): radial depth error sigma = depth_k_true *
rho^2, tangential 0.003 rad, box-bottom edge noise in pixels, 30 Hz, 5 % dropout. The camera
IMU leans toward apparent gravity in turns and surges (``lean``, low-passed) and has a mount
offset; the horizon fix is truth plus noise and a shore dip, at 15 Hz with dropouts.

Metrics, per range bin, against truth: radial and bearing error of each lifted measurement and
of each confirmed track, ids per buoy (1.0 is ideal), share of buoy-frames MOVING, and whether
a 0.8 m/s phantom boat is MOVING.
"""

import argparse
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mhseals_nav import waterline  # noqa: E402, I100
from mhseals_nav.horizon import rotation  # noqa: E402
from mhseals_nav.track_filter import MOTION_MOVING, TrackFilter  # noqa: E402

TRACKER = yaml.safe_load((ROOT / 'config' / 'tracker.yaml').read_text())[
    '/**']['ros__parameters']
BACKEND = yaml.safe_load((ROOT / 'config' / 'zed_backend.yaml').read_text())[
    '/**']['ros__parameters']
IMAGE = (1280, 720)
HFOV = 110.0
FX = IMAGE[0] / 2 / math.tan(math.radians(HFOV / 2))
K = (FX, FX, IMAGE[0] / 2, IMAGE[1] / 2)
T_BC = np.array([0.33, 0.0, 0.125])        # base_link -> camera (URDF)
WATER_Z = -0.20                            # waterline below base_link (perception_costmap)
CLASSES = ['red_buoy', 'green_buoy', 'yellow_buoy', 'black_buoy']
RINGS = (6.0, 10.0, 14.0, 18.0, 22.0)
BINS = ((5, 10), (10, 15), (15, 20), (20, 25))
BUOY_R = 0.2


def field():
    out = []
    for k, radius in enumerate(RINGS):
        for j in range(5):
            a = 2 * math.pi * j / 5 + k * 0.6
            out.append((CLASSES[(j + k) % 4], radius * math.cos(a), radius * math.sin(a)))
    return out


BUOYS = field()


class Plant:
    def __init__(self, seed, duration, dt=1e-3, yaw_max=0.6, ramp=0.8):
        rng = np.random.default_rng(seed)
        self.t = np.arange(0.0, duration + 2.0, dt)
        r = np.zeros_like(self.t)
        t0, level = 0.0, 0.0
        while t0 < self.t[-1]:
            hold = rng.uniform(2.5, 5.0)
            new = rng.choice([-1.0, -0.5, 0.5, 1.0]) * yaw_max
            seg = (self.t >= t0) & (self.t < t0 + ramp + hold)
            f = np.clip((self.t[seg] - t0) / ramp, 0, 1)
            r[seg] = level + (new - level) * f
            t0, level = t0 + ramp + hold, new
        self.speed = 1.5
        # Steer back toward the field centre: keeps the boat within ~4 m of it.
        yaw, x, y = np.zeros_like(self.t), np.zeros_like(self.t), np.zeros_like(self.t)
        for i in range(1, len(self.t)):
            home = math.atan2(-y[i - 1], -x[i - 1])
            pull = 0.8 * math.sin(home - yaw[i - 1]) * min(math.hypot(x[i - 1], y[i - 1]) / 4, 1)
            r[i - 1] = r[i - 1] * 0.6 + pull
            yaw[i] = yaw[i - 1] + r[i - 1] * dt
            x[i] = x[i - 1] + self.speed * math.cos(yaw[i - 1]) * dt
            y[i] = y[i - 1] + self.speed * math.sin(yaw[i - 1]) * dt
        self.r, self.yaw, self.x, self.y = r, yaw, x, y
        ph = rng.uniform(0, 2 * math.pi, 4)
        tt = self.t
        heel = np.radians(1.0) * (self.speed * r) / 0.9
        self.roll = np.radians(4.0) * np.sin(2 * math.pi * 0.35 * tt + ph[0]) + \
            np.radians(1.5) * np.sin(2 * math.pi * 0.9 * tt + ph[1]) + heel
        self.pitch = np.radians(2.5) * np.sin(2 * math.pi * 0.5 * tt + ph[2]) + \
            np.radians(1.0) * np.sin(2 * math.pi * 1.1 * tt + ph[3])
        self.heave = 0.03 * np.sin(2 * math.pi * 0.5 * tt + ph[0])
        # Apparent-gravity lean of an IMU attitude filter: lateral v*r and surge accel / g.
        self.lat = self.speed * r / 9.81
        self.lon = np.zeros_like(tt)

    def interp(self, a, t):
        return float(np.interp(t, self.t, a))

    def truth(self, t):
        return {k: self.interp(getattr(self, k), t)
                for k in ('x', 'y', 'yaw', 'roll', 'pitch', 'heave', 'r')}


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TF:
    """EKF TF: yaw sampled at imu_hz (jittered), held, delayed; no roll or pitch."""

    def __init__(self, plant, imu_hz, delay, seed, horizon_s):
        rng = np.random.default_rng(seed + 1)
        period = 1.0 / imu_hz
        n = int(plant.t[-1] / period) + 2
        jitter = rng.uniform(0, 0.8 * period, n) if imu_hz < 10 else np.zeros(n)
        self.sample_t = np.sort(np.arange(n) * period + jitter)
        self.sample_yaw = np.interp(self.sample_t, plant.t, plant.yaw)
        self.delay, self.plant, self.horizon_s = delay, plant, horizon_s
        self.now = 0.0

    def yaw(self, t):
        i = np.searchsorted(self.sample_t, t - self.delay, side='right') - 1
        return float(self.sample_yaw[max(i, 0)])

    def pose_at(self, t, frame):
        if t > self.now - self.horizon_s or t < 0:
            raise LookupError(t)
        p = self.plant
        R = rz(self.yaw(t))
        base = np.array([p.interp(p.x, t), p.interp(p.y, t), 0.0])
        if frame == 'base_link':
            return R, base
        return R, R @ T_BC + base


def true_camera(tr):
    """World <- camera (rotation, origin) with roll and pitch."""
    Rw = rz(tr['yaw']) @ rotation(tr['roll'], tr['pitch'])
    origin = np.array([tr['x'], tr['y'], tr['heave']]) + Rw @ T_BC
    return Rw, origin


def project(pc):
    return K[2] - K[0] * pc[1] / pc[0], K[3] - K[1] * pc[2] / pc[0]


def lowpass(a, dt, tau):
    out = np.empty_like(a)
    acc, g = 0.0, 1 - math.exp(-dt / tau)
    for i, v in enumerate(a):
        acc += g * (v - acc)
        out[i] = acc
    return out


def run(scn):
    """One session -> metrics dict. ``scn`` keys documented in SCENARIOS."""
    rng = np.random.default_rng(scn['seed'] + 7)
    plant = Plant(scn['seed'], scn['duration'])
    tf = TF(plant, scn['imu_hz'], scn['tf_delay'], scn['seed'], scn['tf_ahead'])
    cfg = copy.deepcopy(TRACKER)
    cfg['meas'].update(scn['meas'])
    cfg['fov'] = {'hfov_deg': HFOV, 'range_m': scn['range_m']}
    tracker = TrackFilter(cfg)
    # The backend's edge sigma is in published (HD720 / 2) pixels; the bench projects at HD720.
    wl = dict(BACKEND['waterline'], camera_height_m=T_BC[2] - WATER_Z,
              edge_sigma_px=2 * BACKEND['waterline']['edge_sigma_px'])
    att = waterline.Attitude(BACKEND['attitude'])
    dt = plant.t[1] - plant.t[0]
    lean = scn['lean']
    imu_roll = plant.roll + lean * lowpass(plant.lat, dt, 1.0) + math.radians(0.3)
    imu_pitch = plant.pitch + lean * lowpass(plant.lon, dt, 1.0) + math.radians(-0.4)
    phantom = {'centre': (0.0, 0.0), 'radius': 9.0, 'speed': 0.8}
    meas_err, track_err, ids, moving, frames, phantom_moving = [], [], {}, {}, {}, []
    mono_err = []    # (true range, mono error, stereo radial error, mono sigma)
    rate = scn['rate']
    next_imu, next_hz = 0.0, 0.0
    for k in range(int(scn['duration'] * rate)):
        t = 1.0 + k / rate
        tr = plant.truth(t)
        while next_imu <= t + 0.03:
            att.add_imu(next_imu, np.interp(next_imu, plant.t, imu_roll) +
                        rng.normal(0, math.radians(0.05)),
                        np.interp(next_imu, plant.t, imu_pitch) +
                        rng.normal(0, math.radians(0.05)))
            next_imu += 1 / 200.0
        while next_hz <= t:
            if rng.random() > scn['horizon_drop']:
                h = plant.truth(next_hz)
                att.add_horizon(next_hz, h['roll'] + rng.normal(0, scn['horizon_sigma']),
                                h['pitch'] + scn['horizon_dip'] +
                                rng.normal(0, scn['horizon_sigma']), scn['horizon_sigma'])
            next_hz += 1 / 15.0
        Rw, origin = true_camera(tr)
        a = phantom['speed'] * t / phantom['radius']
        targets = [(c, x, y, BUOY_R) for c, x, y in BUOYS] + [
            ('yellow_target_boat', phantom['radius'] * math.cos(a),
             phantom['radius'] * math.sin(a), 0.5)]
        obs = []
        visible = []
        for idx, (cls, x, y, radius) in enumerate(targets):
            centre = np.array([x, y, WATER_Z + 0.15])
            pc = Rw.T @ (centre - origin)
            rho = math.hypot(pc[0], pc[1])
            if pc[0] <= 0.5 or rho > scn['range_m'] or \
                    abs(math.degrees(math.atan2(pc[1], pc[0]))) > HFOV / 2 - 3:
                continue
            if rng.random() < 0.05:
                continue
            u = pc / np.linalg.norm(pc)
            noisy = pc + u * rng.normal(0, scn['depth_k_true'] * rho ** 2) + \
                np.cross(u, [0, 0, 1]) * rng.normal(0, 0.003 * rho)
            # Waterline contact: nearest point of the waterline circle, seen by the true camera.
            horiz = centre[:2] - origin[:2]
            contact = np.array([*(centre[:2] - radius * horiz / np.linalg.norm(horiz)),
                                WATER_Z])
            cu, cv = project(Rw.T @ (contact - origin))
            cv += rng.normal(0, scn['edge_px'])
            ob = SimpleNamespace(
                class_id=cls, score=0.9, backend_id='', velocity_kind=0,
                position=SimpleNamespace(x=noisy[0], y=noisy[1], z=noisy[2]),
                position_covariance=[0.0] * 9, size=SimpleNamespace(x=0.4, y=0.4, z=0.5),
                velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0), velocity_covariance=[0.0] * 9,
                has_box2d=True, box2d=[0.0] * 4, mono_range=0.0, mono_range_sigma=0.0)
            est = None
            if scn['attitude'] == 'tf':           # level camera: TF has no roll or pitch
                est = (0.0, 0.0, math.radians(3.0))
            elif scn['attitude'] == 'imu':
                imu = att.imu_at(t)
                if imu is not None:
                    est = (imu[0], imu[1], math.radians(BACKEND['attitude']['imu_sigma_deg']))
            elif scn['attitude'] == 'horizon':
                fused = att.at(t)
                est = fused[:3] if fused is not None else None
            if est is not None:
                mono = waterline.mono_range((cu - 5, cv - 20, cu + 5, cv), K, est[0], est[1],
                                            est[2], wl, half_depth=radius)
                if mono is not None:
                    ob.mono_range, ob.mono_range_sigma = mono
                    if idx < len(BUOYS):
                        true_rho = float(np.hypot(*(centre[:2] - origin[:2])))
                        mono_err.append((true_rho, mono[0] - true_rho,
                                         math.hypot(noisy[0], noisy[1]) - rho, mono[1]))
            obs.append(ob)
            visible.append((idx, np.array([x, y]), ob))
        tf.now = t + scn['latency']
        array = SimpleNamespace(
            header=SimpleNamespace(frame_id='front_camera_link', stamp=SimpleNamespace(
                sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)),
            backend='bench', observations=obs)
        # Measurement-level error: the lifted (and fused) observation against truth.
        try:
            R, p = tf.pose_at(t, 'front_camera_link')
        except LookupError:
            R = None
        if R is not None:
            for idx, xy, ob in visible:
                if idx >= len(BUOYS):
                    continue
                z, _, _ = tracker._measurement(ob, R, p, p, np.zeros(3), 0.0)
                meas_err.append(error_row(z[:2], xy, origin[:2]))
        tracker.step(array, t + scn['latency'], tf.pose_at)
        for track in tracker.tracks:
            d = [math.hypot(track.x[0] - x, track.x[1] - y) if c == track.class_id else 1e9
                 for c, x, y, _ in targets]
            i = int(np.argmin(d))
            if d[i] > 3.0:
                continue
            if i == len(BUOYS):
                phantom_moving.append(track.motion == MOTION_MOVING)
                continue
            ids.setdefault(i, set()).add(track.id)
            frames[i] = frames.get(i, 0) + 1
            moving[i] = moving.get(i, 0) + (track.motion == MOTION_MOVING)
            track_err.append(error_row(track.x[:2], np.array(targets[i][1:3]), origin[:2]))
    out = summarize(meas_err, track_err, ids, moving, frames, phantom_moving, tracker)
    out['mono_vs_stereo'] = {}
    for lo, hi in BINS:
        sel = np.array([r for r in mono_err if lo <= r[0] < hi])
        if len(sel):
            out['mono_vs_stereo'][f'{lo}-{hi}'] = (
                len(sel), float(np.sqrt(np.mean(sel[:, 1] ** 2))),
                float(np.sqrt(np.mean(sel[:, 2] ** 2))), float(np.sqrt(np.mean(sel[:, 3] ** 2))))
    return out


def error_row(est, truth, cam):
    off = truth - cam
    rho = float(np.hypot(*off))
    u = off / rho
    n = np.array([-u[1], u[0]])
    d = est - truth
    return rho, float(d @ u), math.degrees(float(d @ n) / rho)


def binned(rows):
    out = {}
    for lo, hi in BINS:
        sel = [r for r in rows if lo <= r[0] < hi]
        if sel:
            a = np.array(sel)
            out[f'{lo}-{hi}'] = (len(sel), float(np.sqrt(np.mean(a[:, 1] ** 2))),
                                 float(np.sqrt(np.mean(a[:, 2] ** 2))))
    return out


def summarize(meas_err, track_err, ids, moving, frames, phantom_moving, tracker):
    seen = list(ids)
    return {
        'meas': binned(meas_err), 'track': binned(track_err),
        'ids_per_buoy': float(np.mean([len(ids[b]) for b in seen])) if seen else 0.0,
        'extra_ids': int(sum(len(ids[b]) - 1 for b in seen)),
        'buoys': len(seen),
        'moving_share': float(sum(moving.values()) / max(sum(frames.values()), 1)),
        'moving_buoys': int(sum(1 for b in moving if moving[b])),
        'phantom_moving': float(np.mean(phantom_moving[len(phantom_moving) // 4:]))
        if phantom_moving else float('nan'),
        'mono': dict(tracker.mono), 'yaw_rate_from': dict(tracker.yaw_rate_from),
    }


BEFORE_MEAS = {'lag_sigma_s': 0.5, 'mono_fusion': False, 'yaw_rate_backward': False}


def scenario(**kw):
    base = {'seed': 1, 'duration': 120.0, 'rate': 30.0, 'range_m': 25.0, 'imu_hz': 100.0,
            'tf_delay': 0.03, 'tf_ahead': 0.04, 'latency': 0.07, 'depth_k_true': 0.008,
            'edge_px': 2.0, 'lean': 0.3, 'horizon_sigma': math.radians(0.1),
            'horizon_dip': math.radians(0.06), 'horizon_drop': 0.2, 'attitude': 'none',
            'meas': dict(BEFORE_MEAS)}
    base.update(kw)
    return base


def matrix(quick=False):
    dur = 40.0 if quick else 120.0
    after = {'lag_sigma_s': TRACKER['meas']['lag_sigma_s'],
             'mono_fusion': True, 'yaw_rate_backward': True}
    rows = []
    for imu_hz in (4.0, 100.0):
        rows.append((f'before imu{imu_hz:g}', scenario(imu_hz=imu_hz, duration=dur)))
        rows.append((f'after-retune imu{imu_hz:g}', scenario(
            imu_hz=imu_hz, duration=dur, meas=dict(after, mono_fusion=False))))
        for att in ('tf', 'imu', 'horizon'):
            rows.append((f'after mono-{att} imu{imu_hz:g}', scenario(
                imu_hz=imu_hz, duration=dur, attitude=att, meas=after)))
    return rows


def fmt(name, m):
    cells = ' '.join(f'{b}:{v[1]:.2f}m/{v[2]:.2f}deg' for b, v in m['track'].items())
    mcells = ' '.join(f'{b}:{v[1]:.2f}m' for b, v in m['meas'].items())
    return (f"{name:28s} ids/buoy {m['ids_per_buoy']:.2f} (+{m['extra_ids']}) "
            f"moving {m['moving_share']:.3f} ({m['moving_buoys']} buoys) "
            f"phantom {m['phantom_moving']:.2f} | track {cells} | meas {mcells} | "
            f"mono {m['mono']} yaw {m['yaw_rate_from']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--seeds', type=int, default=1)
    ap.add_argument('--only', default='')
    args = ap.parse_args()
    for name, scn in matrix(args.quick):
        if args.only and args.only not in name:
            continue
        for seed in range(1, args.seeds + 1):
            print(fmt(f'{name} s{seed}', run(dict(scn, seed=seed))), flush=True)


if __name__ == '__main__':
    main()
