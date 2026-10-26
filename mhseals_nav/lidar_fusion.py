#!/usr/bin/env python3
"""Lidar range adjustment of camera observations (.handoff/state/lidar-fusion-design.md rev 2).

/perception/observations + /points + camera_info -> /perception/observations_fused: same
header and order, each boxed observation re-ranged along its own camera ray by the lidar
cluster it owns (a merged box may split), and every observation carries its full covariance
in ``position_covariance`` (the tracker then runs with ``meas.depth_term: false``).

Everything above the node class is pure numpy. Internally points
and observations live in optical axes (X right, Y down, Z forward); ``A`` maps the camera
frame's axes into them.
"""

from collections import deque
import json
import math
import time

import numpy as np

DEG = math.pi / 180
UP = np.array([0.0, -1.0, 0.0])                  # optical axes
BODY_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
SOURCES = ('lidar', 'split', 'held', 'stereo', 'pass')


# ------------------------------------------------------------------ geometry
def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_matrix(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll); arrays broadcast to (..., 3, 3)."""
    roll, pitch, yaw = np.broadcast_arrays(*(np.asarray(a, float) for a in (roll, pitch, yaw)))
    cr, sr, cp, sp, cy, sy = (f(a) for a in (roll, pitch, yaw) for f in (np.cos, np.sin))
    R = np.empty(roll.shape + (3, 3))
    R[..., 0, :] = np.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], -1)
    R[..., 1, :] = np.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], -1)
    R[..., 2, :] = np.stack([-sp, cp * sr, cp * cr], -1)
    return R


def matrix_rpy(R):
    return (math.atan2(R[2][1], R[2][2]), -math.asin(max(-1.0, min(1.0, R[2][0]))),
            math.atan2(R[1][0], R[0][0]))


def small_rotation(w):
    """Rotation of the vector w (rad), Rodrigues."""
    w = np.asarray(w, float)
    a = float(np.linalg.norm(w))
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * Kx + (1 - math.cos(a)) * Kx @ Kx


def axes_matrix(frame, axes):
    """Optical-from-frame rotation: ``auto`` reads 'optical' in the frame name (as the tracker)."""
    if axes == 'auto':
        axes = 'optical' if 'optical' in frame else 'body'
    return np.eye(3) if axes == 'optical' else BODY_TO_OPTICAL


def delta_knots(poses, c):
    """Knot params (K, 6) [x y z roll pitch yaw] of T(t_c)^-1 T(t_k); yaw unwrapped."""
    Rc, pc = c
    out = []
    for R, p in poses:
        D = Rc.T @ R
        out.append([*(Rc.T @ (p - pc)), *matrix_rpy(D)])
    out = np.array(out)
    out[:, 3:] = np.unwrap(out[:, 3:], axis=0)
    return out


def compensate(points, t, knot_t, knots):
    """Base-frame points seen at their own t_i -> base frame at t_c (static world)."""
    q = np.stack([np.interp(t, knot_t, knots[:, j]) for j in range(6)], axis=1)
    R = rpy_matrix(q[:, 3], q[:, 4], q[:, 5])
    return np.einsum('nij,nj->ni', R, points) + q[:, :3]


def cloud_arrays(msg, r_min):
    """(xyz (N, 3), tau (N,)) of finite points at range >= r_min; tau 0 without a time field."""
    fields = {f.name: f for f in msg.fields}
    count = msg.width * msg.height
    order = '>' if msg.is_bigendian else '<'
    timed = 'time' in fields and fields['time'].datatype == 7      # FLOAT32
    names = ['x', 'y', 'z'] + (['time'] if timed else [])
    layout = np.dtype({'names': names, 'formats': [order + 'f4'] * len(names),
                       'offsets': [fields[n].offset for n in names],
                       'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=layout, count=count)
    xyz = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(float)
    tau = raw['time'].astype(float) if 'time' in names else np.zeros(count)
    keep = np.isfinite(xyz).all(axis=1) & np.isfinite(tau) & \
        (np.einsum('ij,ij->i', xyz, xyz) >= r_min ** 2)
    return xyz[keep], tau[keep]


def crop_mask(points_b, cam_t, cam_forward, half_angle):
    """Points whose base-frame azimuth about the camera is within half_angle of its forward."""
    az = np.arctan2(points_b[:, 1] - cam_t[1], points_b[:, 0] - cam_t[0]) - \
        math.atan2(cam_forward[1], cam_forward[0])
    return np.abs((az + math.pi) % (2 * math.pi) - math.pi) < half_angle


def choose_scan(scans, t_c, max_dt):
    """Pick the scan whose sector median time is nearest t_c, if within max_dt."""
    best = min(scans, key=lambda s: abs(s['t_med'] - t_c), default=None)
    return best if best is not None and abs(best['t_med'] - t_c) <= max_dt else None


def tf_gap(stamps, lo, hi):
    """Largest spacing of the TF samples that bracket [lo, hi]; inf when they do not."""
    s = np.asarray(stamps, float)
    if len(s) < 2:
        return math.inf
    i0 = int(np.searchsorted(s, lo, side='right')) - 1
    i1 = int(np.searchsorted(s, hi, side='left'))
    if i0 < 0 or i1 >= len(s):
        return math.inf
    return float(np.max(np.diff(s[i0:i1 + 1]))) if i1 > i0 else 0.0


def project(P, K):
    fx, fy, cx, cy = K
    return fx * P[:, 0] / P[:, 2] + cx, fy * P[:, 1] / P[:, 2] + cy


def covariance(p, sigma_r2, sigma_beta, sigma_z):
    """Optical-axes 3x3: radial, tangential (rho*sigma_beta) and vertical parts (design §2)."""
    rho = max(math.hypot(p[0], p[2]), 1e-6)
    u = np.array([p[0], 0.0, p[2]]) / rho
    n = np.cross(UP, u)
    return sigma_r2 * np.outer(u, u) + (rho * sigma_beta) ** 2 * np.outer(n, n) + \
        sigma_z ** 2 * np.outer(UP, UP)


# --------------------------------------------------------------- per-frame state
class RateEma:
    """Observation rate per backend_id ('' = the whole stream): EMA of 1/dt."""

    def __init__(self, alpha, default_hz):
        self.alpha, self.default, self.last, self.hz = alpha, default_hz, {}, {}

    def update(self, key, t):
        last = self.last.get(key)
        self.last[key] = t
        if last is not None and t > last:
            f = self.hz.get(key, self.default)
            self.hz[key] = f + self.alpha * (1.0 / (t - last) - f)
        return self.hz.get(key, self.default)


class HeldRatio:
    """lidar/stereo range ratio per backend_id, held for stereo-only frames (design §2)."""

    def __init__(self, alpha, hold_s, tau_s):
        self.alpha, self.hold_s, self.tau_s, self.ids = alpha, hold_s, tau_s, {}

    def lidar(self, key, t, ratio, sigma_r2):
        h = self.ids.get(key)
        if h is None or t - h['t'] > self.hold_s:
            self.ids[key] = {'k': ratio, 'var': 0.0, 't': t, 's2': sigma_r2}
            return
        d = ratio - h['k']
        h['k'] += self.alpha * d
        h['var'] = (1 - self.alpha) * (h['var'] + self.alpha * d * d)
        h['t'], h['s2'] = t, sigma_r2

    def held(self, key, t, rho_s, depth_k):
        """(k, sigma_r^2) for a stereo-only frame within hold_s of the last match, else None."""
        h = self.ids.get(key) if key else None
        if h is None or not 0 <= t - h['t'] <= self.hold_s:
            return None
        age = t - h['t']
        s_age = depth_k * rho_s ** 2 * min(1.0, math.sqrt(2 * (1 - math.exp(-age / self.tau_s))))
        return h['k'], h['s2'] + rho_s ** 2 * h['var'] + s_age ** 2

    def clear(self):
        self.ids.clear()


# ------------------------------------------------------------------- boxes
class Box:
    """One observation with what association needs, in optical axes and pixels."""

    def __init__(self, index, ob, ctx):
        self.index, self.ob = index, ob
        fx, fy, cx, cy = ctx.K
        W, H = ctx.image
        self.s = ctx.A @ np.array([ob.position.x, ob.position.y, ob.position.z])
        self.ok = bool(np.all(np.isfinite(self.s))) and self.s[2] > 0
        self.rho = math.hypot(self.s[0], self.s[2]) if self.ok else math.nan
        self.radius = ctx.radius(ob.class_id)
        cov = ctx.A @ np.array(ob.position_covariance, float).reshape(3, 3) @ ctx.A.T
        self.has_cov = bool(np.any(cov))
        self.boxed = bool(ob.has_box2d) and self.ok
        if self.ok:
            m = ctx.meas
            u = np.array([self.s[0], 0.0, self.s[2]]) / self.rho
            n = np.cross(UP, u)
            floor = max(m['min_sigma_m'], m['range_frac'] * self.rho) ** 2
            self.s_st2 = float(u @ cov @ u) if self.has_cov else floor
            self.tan2 = float(n @ cov @ n) if self.has_cov else floor
            self.sz = math.sqrt(float(UP @ cov @ UP)) if self.has_cov else ctx.cfg['sigma_z_m']
            self.sigma_s = math.sqrt(self.s_st2 + (m['depth_k'] * self.rho ** 2) ** 2)
        if self.boxed:
            b = ob.box2d
            self.u0, self.v0, self.u1, self.v1 = b[0] * W, b[1] * H, b[2] * W, b[3] * H
            self.w = self.u1 - self.u0
            x = ((self.u0 + self.u1) / 2 - cx) / fx
            self.e_u = fx * (1 + x * x) * math.tan(ctx.cfg['delta_h_deg'] * DEG + ctx.e_mc)
            self.e_v = fy * math.tan(ctx.cfg['margin_v_deg'] * DEG)
            self.w_exp = fx * 2 * self.radius / self.rho


def iou_x(a0, a1, b0, b1):
    inter = min(a1, b1) - max(a0, b0)
    if inter <= 0:
        return 0.0
    return inter / (max(a1, b1) - min(a0, b0))


def clusters(rho, az, cfg):
    """Index arrays: split by range gaps, then by azimuth gaps (design §2 Clusters)."""
    da = cfg['delta_a_rad']
    order = np.argsort(rho)
    r = rho[order]
    cuts = np.nonzero(np.diff(r) > np.maximum(cfg['gap_min_m'], 3 * r[:-1] * da))[0] + 1
    out = []
    for piece in np.split(order, cuts):
        piece = piece[np.argsort(az[piece])]
        gaps = np.nonzero(np.diff(az[piece]) > 3 * da + cfg['divergence_rad'])[0] + 1
        out.extend(p for p in np.split(piece, gaps) if len(p))
    return out


class Cluster:
    def __init__(self, idx, rho, u, v, fx, da):
        self.idx = idx
        self.n = len(idx)
        self.rho = float(np.median(rho[idx]))
        half = fx * da / 2
        self.u0, self.u1 = float(u[idx].min()) - half, float(u[idx].max()) + half
        self.v0, self.v1 = float(v[idx].min()), float(v[idx].max())
        self.vc = float(np.mean(v[idx]))
        self.uc, self.w = (self.u0 + self.u1) / 2, self.u1 - self.u0


def n_floor(cl, box, ctx):
    """max(2, 0.3 n_exp): rings over the box's visible height times beams across 2R."""
    cfg, fy = ctx.cfg, ctx.K[1]
    ring = math.tan(cfg['ring_spacing_deg'] * DEG)
    h_vis = (box.v1 - box.v0) * cl.rho / fy
    rings = max(1, math.floor(h_vis / (cl.rho * ring)))
    n_exp = rings * 2 * box.radius / (cl.rho * cfg['delta_a_rad'])
    return max(2.0, cfg['n_frac'] * n_exp)


def pair_cost(cl, box, ctx):
    """Association cost of a cluster for a box, or None when infeasible (design §2)."""
    cfg, fx = ctx.cfg, ctx.K[0]
    da_px = fx * cfg['delta_a_rad']
    if cl.n < n_floor(cl, box, ctx):
        return None
    iou = iou_x(cl.u0, cl.u1, box.u0 - box.e_u, box.u1 + box.e_u)
    if iou < cfg['iou_min'] or min(cl.v1, box.v1 + box.e_v) - max(cl.v0, box.v0 - box.e_v) <= 0:
        return None
    rho_obj = cl.rho + cfg['near_face_k'] * box.radius
    if abs(rho_obj - box.rho) >= 3 * box.sigma_s + cfg['gate_pad_m']:
        return None
    if cl.w > 2 * max(box.w, box.w_exp) + da_px:
        return None
    return (1 - iou) + 0.5 * abs(rho_obj - box.rho) / (3 * box.sigma_s) + \
        0.5 * abs(cl.w - box.w_exp) / max(box.w_exp, da_px)


class Context:
    """Per-frame constants: intrinsics, axes, config, tracker measurement model, e_mc."""

    def __init__(self, cfg, tracker, K, image, A, e_mc=0.0, se_yaw=0.0):
        self.cfg, self.K, self.image, self.A = cfg, K, image, A
        self.meas, self.radii = tracker['meas'], tracker['radius']
        self.e_mc, self.se_yaw = e_mc, se_yaw

    def radius(self, class_id):
        return float(self.radii.get(class_id, self.radii['default']))


def candidates(boxes, P, u, v, rho):
    """Mask of in-image points that fall in some box's widened window and range gate."""
    keep = np.zeros(len(P), bool)
    for b in boxes:
        keep |= (u >= b.u0 - b.e_u) & (u <= b.u1 + b.e_u) & (v >= b.v0 - b.e_v) & \
            (v <= b.v1 + b.e_v) & (np.abs(rho - b.rho) < 3 * b.sigma_s + b.radius + 0.3)
    return keep


def assignment(C):
    """Minimum-cost one-to-one rows -> columns (Hungarian, e-maxx potentials), any shape.

    Same optimum as scipy's linear_sum_assignment, without scipy: its apt build breaks under
    the pip numpy 2 of the ROS images.
    """
    flip = C.shape[0] > C.shape[1]
    A = C.T if flip else C
    n, m = A.shape
    u, v, p, way = np.zeros(n + 1), np.zeros(m + 1), np.zeros(m + 1, int), np.zeros(m + 1, int)
    for i in range(1, n + 1):
        p[0], j0 = i, 0
        minv, used = np.full(m + 1, np.inf), np.zeros(m + 1, bool)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], np.inf, 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = A[i0 - 1, j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    pairs = sorted((p[j] - 1, j - 1) for j in range(1, m + 1) if p[j])
    return [(c, r) for r, c in pairs] if flip else pairs


def associate(boxes, cls, ctx):
    """Per box: ('lidar', [c]) | ('split', [c...]) | ('ambiguous', []) | ('none', [])."""
    cfg, fx = ctx.cfg, ctx.K[0]
    da_px = fx * cfg['delta_a_rad']
    nb, nc = len(boxes), len(cls)
    C = np.full((nb, nc), np.inf)
    for i, b in enumerate(boxes):
        for j, c in enumerate(cls):
            cost = pair_cost(c, b, ctx)
            if cost is not None:
                C[i, j] = cost
    feasible = np.isfinite(C)
    owner = {}
    if nb and nc:
        rows, cols = zip(*assignment(np.where(feasible, C, 1e6)))
        owner = {i: j for i, j in zip(rows, cols)
                 if feasible[i, j] and C[i, j] <= cfg['accept_cost']}
    taken = set(owner.values())
    out = []
    for i, b in enumerate(boxes):
        if b.w > cfg['merge_w_frac'] * b.w_exp:
            sized = [j for j in np.nonzero(feasible[i])[0]
                     if (j not in taken or owner.get(i) == j)
                     and cls[j].w <= 1.5 * fx * 2 * b.radius / cls[j].rho + da_px]
            cover = sum(max(0.0, min(cls[j].u1, b.u1) - max(cls[j].u0, b.u0)) for j in sized)
            if len(sized) >= 2 and cover >= 0.6 * b.w:
                out.append(('split', sorted(sized, key=lambda j: cls[j].uc)))
                continue
        j = owner.get(i)
        if j is None:
            out.append(('none', []))
            continue
        c1, dc = C[i, j], cfg['ambiguity_cost']
        second = any(C[i, k] - c1 < dc for k in np.nonzero(feasible[i])[0] if k != j)
        contested = any(C[k, j] < c1 + dc for k in np.nonzero(feasible[:, j])[0] if k != i)
        out.append(('ambiguous', []) if second or contested else ('lidar', [j]))
    return out


def lidar_sigma_r2(cfg, radius, f_obs):
    """sigma_w^2 + n_b sigma_b^2 with n_b = f_obs T_b (bias-aware, design §2 Covariance)."""
    sb2 = (cfg['bias_radius_frac'] * radius) ** 2 + cfg['sigma_t_ext_m'] ** 2
    return cfg['sigma_w_m'] ** 2 + f_obs * cfg['bias_time_s'] * sb2


def adjust(obs, P, ctx, t, rates=None, held=None, use_lidar=True):
    """Fuse one frame. Returns [(source, index, position (frame axes), cov9, box2d|None, id)].

    ``P`` (N, 3) are motion-compensated, extrinsic-corrected lidar points in optical axes.
    Output keeps the input order; a split's extra entries follow their source observation.
    ``rates`` (RateEma) and ``held`` (HeldRatio) carry state across frames when given.
    """
    cfg, (fx, _, cx, _) = ctx.cfg, ctx.K
    W, H = ctx.image
    rates = rates or RateEma(cfg['rate_alpha'], cfg['default_hz'])
    held = held or HeldRatio(cfg['held_alpha'], cfg['held_s'], cfg['held_tau_s'])
    boxes = [Box(i, ob, ctx) for i, ob in enumerate(obs)]
    usable = [b for b in boxes if b.boxed]
    decisions = {}
    cls = []
    if use_lidar and usable and len(P):
        u, v = project(P, K=ctx.K)
        rho = np.hypot(P[:, 0], P[:, 2])
        keep = candidates(usable, P, u, v, rho)
        idx = np.nonzero(keep)[0]
        az = np.arctan2(P[idx, 0], P[idx, 2])
        for part in clusters(rho[idx], az, cfg):
            cls.append(Cluster(idx[part], rho, u, v, fx, cfg['delta_a_rad']))
        for b, d in zip(usable, associate(usable, cls, ctx)):
            decisions[b.index] = d
    out = []
    Ainv = ctx.A.T

    def emit(source, b, p, s_r2, s_beta, box2d=None, key=None, tan2=None):
        Sigma = covariance(p, s_r2, s_beta, b.sz)
        if tan2 is not None:      # box-less: keep the stereo tangential variance
            rho_p = max(math.hypot(p[0], p[2]), 1e-6)
            n = np.cross(UP, np.array([p[0], 0.0, p[2]]) / rho_p)
            Sigma += (tan2 - (rho_p * s_beta) ** 2) * np.outer(n, n)
        out.append((source, b.index, Ainv @ p, (Ainv @ Sigma @ ctx.A).reshape(-1).tolist(),
                    box2d, b.ob.backend_id if key is None else key))

    for b in boxes:
        if not b.ok:
            out.append(('pass', b.index, None, None, None, b.ob.backend_id))
            continue
        key = b.ob.backend_id
        f_obs = rates.update(key, t)
        if not b.boxed:
            emit('stereo', b, b.s, b.sigma_s ** 2, 0.0, tan2=b.tan2)
            continue
        beta = (cfg['box_beta_frac'] * b.w + 1.0) / fx
        kind, js = decisions.get(b.index, ('none', []))
        if kind == 'lidar':
            c = cls[js[0]]
            rho_obj = c.rho + cfg['near_face_k'] * b.radius
            s_r2 = lidar_sigma_r2(cfg, b.radius, f_obs)
            if key:
                held.lidar(key, t, rho_obj / b.rho, s_r2)
            emit('lidar', b, b.s * (rho_obj / b.rho), s_r2, beta)
        elif kind == 'split':
            az_s = math.atan2(b.s[0], b.s[2])
            parts = []
            for j in js:
                c = cls[j]
                rho_obj = c.rho + cfg['near_face_k'] * b.radius
                az = math.atan((c.uc - cx) / fx)
                p = np.array([rho_obj * math.sin(az), b.s[1] * rho_obj / b.rho,
                              rho_obj * math.cos(az)])
                parts.append((abs(az - az_s), c, p))
            nearest = min(range(len(parts)), key=lambda k: parts[k][0])
            for k, (_, c, p) in enumerate(parts):
                beta2 = ((cfg['box_beta_frac'] * c.w + 1.0) / fx) ** 2 + \
                    cfg['delta_a_rad'] ** 2 / 12 + ctx.se_yaw ** 2 + cfg['split_beta_rad'] ** 2
                box2d = [max(0.0, c.u0 / W), b.v0 / H, min(1.0, c.u1 / W), b.v1 / H]
                emit('split', b, p, lidar_sigma_r2(cfg, b.radius, f_obs),
                     math.sqrt(beta2), box2d, key if k == nearest else '')
        else:
            h = held.held(key, t, b.rho, ctx.meas['depth_k'])
            if h is not None:
                emit('held', b, b.s * h[0], h[1], beta)
            else:
                emit('stereo', b, b.s, b.sigma_s ** 2, beta)
    return out, cls


# ------------------------------------------------------------ extrinsic estimate
def extrinsic_samples(obs, P, ctx):
    """Isolated, near, unclipped boxes and their single raw-projection cluster (if exactly one).

    ``P`` is the raw projection (no applied correction), so a wrong correction can not
    confirm itself. Returns [(box, cluster or None)] over the qualifying boxes.
    """
    cfg, (fx, fy, _, _) = ctx.cfg, ctx.K
    W, H = ctx.image
    boxes = [b for b in (Box(i, ob, ctx) for i, ob in enumerate(obs)) if b.boxed]
    iso = fx * math.tan(cfg['isolation_deg'] * DEG)
    edge = cfg['border_px']
    out = []
    if not len(P):
        P = np.zeros((0, 3))
    u, v = project(P, ctx.K) if len(P) else (np.zeros(0), np.zeros(0))
    rho = np.hypot(P[:, 0], P[:, 2])
    win = math.tan((cfg['sample_window_deg'] + cfg['delta_h_deg']) * DEG)
    for b in boxes:
        if b.rho >= cfg['sample_max_range_m'] or b.u0 < edge or b.v0 < edge or \
                b.u1 > W - edge or b.v1 > H - edge:
            continue
        if any(o is not b and min(o.u1, b.u1 + iso) - max(o.u0, b.u0 - iso) > 0 for o in boxes):
            continue
        sel = np.nonzero((u >= b.u0 - fx * win) & (u <= b.u1 + fx * win) &
                         (v >= b.v0 - fy * win) & (v <= b.v1 + fy * win) &
                         (np.abs(rho - b.rho) < 3 * b.sigma_s + b.radius + 0.3))[0]
        az = np.arctan2(P[sel, 0], P[sel, 2])
        found = [Cluster(sel[p], rho, u, v, fx, cfg['delta_a_rad'])
                 for p in clusters(rho[sel], az, cfg)]
        found = [c for c in found if c.n >= n_floor(c, b, ctx)]
        out.append((b, found[0] if len(found) == 1 else None))
    return out


def model_rows(x, y, Z, yaw_rate):
    """Design §2 linear model rows over [w_x, w_y, w_z, c_x, c_y, c_z, tau] for (r_az, r_el)."""
    ax, ay = 1 + x * x, 1 + y * y
    az = [-x * y / ax, 1.0, -y / ax, 1 / (Z * ax), 0.0, -x / (Z * ax), yaw_rate]
    el = [-1.0, x * y / ay, x / ay, 0.0, 1 / (Z * ay), -y / (Z * ay), 0.0]
    return az, el


class ExtrinsicEstimate:
    """Joint WLS of the lidar-camera rotation, translation and image time; detect and alert."""

    OK, WARN, ERROR = 0, 1, 2

    def __init__(self, cfg):
        self.cfg = cfg
        self.samples = deque(maxlen=cfg['window_n'])
        self.last_key = {}
        self.new = 0
        self.theta, self.se = np.zeros(7), np.full(7, np.inf)
        self.applied = np.zeros(3)
        self.level, self.message, self.warn_windows = self.OK, '', 0
        self.alerts = set()
        self.isolated = deque()         # (t, sampled, lidar)

    def add(self, t, key, b, c, ctx, yaw_rate):
        """Add one sample (box b, raw cluster c); at most one per (key, min_dt_s)."""
        cfg, (fx, fy, cx, cy) = self.cfg, ctx.K
        if key and t - self.last_key.get(key, -math.inf) < cfg['sample_dt_s']:
            return False
        self.last_key[key] = t
        x, y = ((b.u0 + b.u1) / 2 - cx) / fx, ((b.v0 + b.v1) / 2 - cy) / fy
        r_az = math.atan((c.uc - cx) / fx) - math.atan(x)
        r_el = math.atan((c.vc - cy) / fy) - math.atan(y)
        s_az = cfg['sigma_u_px'] / (fx * (1 + x * x))
        s_el = (b.v1 - b.v0) / (fy * math.sqrt(12) * (1 + y * y))
        rows = model_rows(x, y, c.rho, yaw_rate)
        self.samples.append((rows, (r_az, r_el), (s_az, max(s_el, 1e-4))))
        self.last = (x, y, c.rho, yaw_rate, r_az, r_el, s_az, max(s_el, 1e-4))
        self.new += 1
        if len(self.samples) >= cfg['min_n'] and self.new >= cfg['stride']:
            self.new = 0
            self.evaluate()
        return True

    def fit(self):
        A, r, w = [], [], []
        for rows, res, sig in self.samples:
            A += rows
            r += res
            w += [1 / s ** 2 for s in sig]
        A, r, w = np.array(A), np.array(r), np.array(w)
        prior = 1 / np.array(self.cfg['prior_sigma']) ** 2
        N = A.T @ (A * w[:, None]) + np.diag(prior)
        cov = np.linalg.inv(N)
        theta = cov @ (A.T @ (w * r))
        chi2 = float(np.sum(w * (r - A @ theta) ** 2))
        dof = max(1, len(r) - len(theta))
        return theta, np.sqrt(np.diag(cov) * max(1.0, chi2 / dof))

    def evaluate(self):
        cfg = self.cfg
        self.theta, self.se = self.fit()
        w, s = self.theta[:3], self.se[:3]
        if self.level == self.ERROR:
            return
        big = np.abs(w) > cfg['error_deg'] * DEG
        if np.any(big & (np.abs(w) > cfg['k_se'] * s)):
            return self.error(f'lidar-camera rotation {np.degrees(w).round(2).tolist()} deg '
                              f'(pitch, yaw, roll) beyond {cfg["error_deg"]} deg')
        if s[1] < cfg['apply_se_deg'] * DEG:
            lim = cfg['apply_max_deg'] * DEG
            self.applied = np.clip(w, -lim, lim)
        rp, rp_se = math.hypot(w[0], w[2]), math.hypot(s[0], s[2])
        warn = (abs(w[1]) > cfg['warn_deg'] * DEG and abs(w[1]) > cfg['k_se'] * s[1]) or \
            (rp > cfg['warn_deg'] * DEG and rp > cfg['k_se'] * rp_se)
        self.warn_windows = self.warn_windows + 1 if warn else 0
        if self.warn_windows >= cfg['warn_windows']:
            self.level = self.WARN
            self.message = f'lidar-camera rotation {np.degrees(w).round(2).tolist()} deg'
        elif not warn and self.level == self.WARN:
            self.level, self.message = self.OK, ''
        c, cs, tau, ts = self.theta[3:5], self.se[3:5], self.theta[6], self.se[6]
        self.alerts = set()
        c_norm = np.linalg.norm(c)
        if c_norm > cfg['alert_c_m'] and c_norm > cfg['k_se'] * np.linalg.norm(cs):
            self.alerts.add('translation')
        if abs(tau) > cfg['alert_tau_s'] and abs(tau) > cfg['k_se'] * ts:
            self.alerts.add('image_time')

    def observe_isolated(self, t, sampled, lidar):
        """Silent failure: too few samples or lidar ranges over many isolated near boxes."""
        cfg = self.cfg
        self.isolated.append((t, sampled, lidar))
        while self.isolated and t - self.isolated[0][0] > cfg['silent_window_s']:
            self.isolated.popleft()
        n = len(self.isolated)
        if self.level == self.ERROR or n < cfg['silent_min_boxes'] or \
                t - self.isolated[0][0] < 0.9 * cfg['silent_window_s']:
            return
        frac_s = sum(s for _, s, _ in self.isolated) / n
        frac_l = sum(ld for _, _, ld in self.isolated) / n
        if min(frac_s, frac_l) < cfg['silent_frac']:
            self.error(f'silent extrinsic failure: samples {frac_s:.0%}, lidar share '
                       f'{frac_l:.0%} of {n} isolated boxes in {cfg["silent_window_s"]:g} s')

    def error(self, message):
        self.level, self.message, self.applied = self.ERROR, message, np.zeros(3)


# ---------------------------------------------------------------------- node
try:
    from rclpy.node import Node
except ImportError:  # host pytest exercises only the pure functions
    Node = object


def load_yaml(name):
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('mhseals_nav')) / 'config' / name
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


class LidarFusion(Node):
    def __init__(self):
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Imu
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener
        from mhseals_nav.msg import ObservationArray
        from mhseals_nav.yaml_params import node_config
        super().__init__('lidar_fusion', automatically_declare_parameters_from_overrides=True)
        self.cfg = cfg = node_config(self, 'lidar_fusion.yaml')
        self.tracker = load_yaml('tracker.yaml')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.K = self.image = None
        self.static = {}                          # frame -> (R, t) base<-frame
        self.scans = deque(maxlen=cfg['scans_kept'])
        import threading
        self.lock = threading.RLock()
        self.queue = deque()
        self.tf_stamps = deque(maxlen=400)        # odom<-base stamps from /tf
        self.attitude = deque()
        self.rates = RateEma(cfg['rate_alpha'], cfg['default_hz'])
        self.held = HeldRatio(cfg['held_alpha'], cfg['held_s'], cfg['held_tau_s'])
        self.extrinsic = ExtrinsicEstimate(cfg['extrinsic'])
        self.counts = dict.fromkeys(SOURCES, 0)
        self.skips = {}
        self.proc = deque(maxlen=200)
        self.cpu_at = (time.monotonic(), time.process_time())
        self.Diag = DiagnosticArray
        lossy = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(ObservationArray, cfg['output_topic'], lossy)
        self.status_pub = self.create_publisher(DiagnosticArray, '/perception/status', 1)
        from std_msgs.msg import String
        self.String = String     # per-frame source labels for the campaign (JSON)
        self.sources_pub = self.create_publisher(String, cfg['sources_topic'], 10)
        self.create_subscription(ObservationArray, cfg['observations_topic'], self.on_frame,
                                 lossy)
        self.create_subscription(CameraInfo, cfg['camera_info_topic'], self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        if cfg['attitude_topic']:
            self.create_subscription(Imu, cfg['attitude_topic'], self.on_attitude,
                                     qos_profile_sensor_data)
        if cfg['points_transport'] == 'direct':
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(PointCloud2, cfg['points_topic'], self.on_cloud,
                                     qos_profile_sensor_data)
        else:
            self.start_relay()
        self.create_timer(0.02, self.drain)
        self.create_timer(1.0, self.report)

    # ------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def start_relay(self):
        """Full-rate /points from the large-data graph through lidar_relay's receiver."""
        import multiprocessing
        import os
        import threading
        from ament_index_python.packages import get_package_share_directory
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from mhseals_nav import lidar_relay
        profile = os.path.join(get_package_share_directory('mhseals_nav'), 'config',
                               'fastdds_large_data.xml')
        context = multiprocessing.get_context('spawn')
        reader, writer = context.Pipe(duplex=False)
        self.child = context.Process(target=lidar_relay.receive, daemon=True,
                                     args=(writer, self.cfg['points_topic'], 0, profile))
        self.child.start()

        def pump():
            while True:
                try:
                    payload = reader.recv_bytes()
                except EOFError:
                    break
                self.on_cloud(deserialize_message(payload, PointCloud2))
            self.get_logger().error('lidar receiver exited')
            os._exit(1)            # respawn restarts both halves
        threading.Thread(target=pump, daemon=True).start()

    def on_camera_info(self, msg):
        if msg.k[0] > 0:
            self.K, self.image = (msg.k[0], msg.k[4], msg.k[2], msg.k[5]), (msg.width, msg.height)

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.cfg['odom_frame'] and \
                    tf.child_frame_id == self.cfg['base_frame']:
                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if not self.tf_stamps or t > self.tf_stamps[-1]:
                    self.tf_stamps.append(t)
                elif t < self.tf_stamps[0] - 1.0:
                    self.tf_stamps.clear()     # clock jumped back (bag loop)
                    self.tf_stamps.append(t)

    def on_attitude(self, _msg):
        now = self.now()
        self.attitude.append(now)
        while self.attitude and now - self.attitude[0] > 2.0:
            self.attitude.popleft()

    def base_from(self, frame):
        """(R, t) base<-frame, cached: a mast swap is a URDF change only."""
        if frame not in self.static:
            from rclpy.time import Time
            from tf2_ros import TransformException
            try:
                tf = self.tf.lookup_transform(self.cfg['base_frame'], frame, Time()).transform
            except TransformException:
                return None
            r, t = tf.rotation, tf.translation
            self.static[frame] = quat_matrix(r.x, r.y, r.z, r.w), np.array([t.x, t.y, t.z])
        return self.static[frame]

    def on_cloud(self, msg):
        T_bl = self.base_from(msg.header.frame_id)
        if T_bl is None:
            return
        xyz, tau = cloud_arrays(msg, self.cfg['r_min_m'])
        pb = xyz @ T_bl[0].T + T_bl[1]
        cam = self.camera
        if cam is not None and self.K is not None:
            half = math.atan(self.image[0] / 2 / self.K[0]) + 5 * DEG + \
                self.cfg['omega_max_rad_s'] * 0.2
            keep = crop_mask(pb, cam[1], cam[0].T[2], half)   # optical forward in base
            pb, tau = pb[keep], tau[keep]
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(tau):
            with self.lock:
                self.scans.append({'t_s': t_s, 't_med': t_s + float(np.median(tau)), 'p': pb,
                                   'tau': tau})
            self.drain()            # frames waiting for this scan

    @property
    def camera(self):
        """(R, t) base<-optical camera, once the camera frame is known."""
        return getattr(self, '_camera', None)

    def set_camera(self, frame):
        T = self.base_from(frame)
        if T is None:
            return None
        A = axes_matrix(frame, self.cfg['camera_axes'])
        self._camera = (T[0] @ A.T, T[1])
        self.A = A
        return self._camera

    def pose_at(self, t):
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tf = self.tf.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'],
                                          Time(nanoseconds=int(t * 1e9))).transform
        except TransformException as error:
            raise LookupError(str(error)) from error
        r, p = tf.rotation, tf.translation
        return quat_matrix(r.x, r.y, r.z, r.w), np.array([p.x, p.y, p.z])

    # ------------------------------------------------------------ frames
    def on_frame(self, msg):
        self.queue.append((msg, time.monotonic()))
        self.drain()

    def drain(self):
        with self.lock:         # the relay's pipe thread drains on each scan too
            while self.queue:
                msg, arrived = self.queue[0]
                out = self.process(msg)
                if out is None:
                    return          # waiting for TF or the scan
                self.queue.popleft()
                self.pub.publish(out)
                t = out.header.stamp.sec + out.header.stamp.nanosec * 1e-9
                # lat: image capture -> publish (ROS time); wait: frame arrival -> publish
                self.sources_pub.publish(self.String(data=json.dumps(
                    {'t': t, 'sources': self.sources, 'skip': self.last_skip,
                     'lat': self.now() - t, 'wait': time.monotonic() - arrived,
                     'ex': self.ex_log})))

    def skip(self, cause):
        self.skips[cause] = self.skips.get(cause, 0) + 1
        self.last_skip = cause

    def process(self, msg):
        """Return the fused array, or None while TF may still arrive (at most tf_wait_s)."""
        started = time.process_time()
        cfg = self.cfg
        self.last_skip, self.ex_log = '', []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_c = stamp + cfg['image_time_offset_s']
        frame = cfg['camera_frame'] or msg.header.frame_id
        use, P, P_raw, yaw_rate, e_mc = False, np.zeros((0, 3)), np.zeros((0, 3)), 0.0, 0.0
        cam = self.camera or self.set_camera(frame)
        if self.K is None or cam is None:
            self.skip('no_camera')
        elif self.extrinsic.level == ExtrinsicEstimate.ERROR:
            self.skip('extrinsic_error')
        elif any(ob.has_box2d for ob in msg.observations):
            scan = choose_scan(self.scans, t_c, cfg['scan_max_dt_s'])
            # A sweep ends (and is stamped) up to 0.1 s after the image it covers: wait for
            # it, as for TF, rather than skip (the campaign skipped 42-60% of smeared frames).
            coming = not self.scans or self.scans[-1]['t_s'] < t_c + cfg['scan_max_dt_s'] / 2
            if scan is None and coming and self.now() - stamp < cfg['tf_wait_s']:
                return None
            if scan is None:
                self.skip('no_scan')
            else:
                t_i = scan['t_s'] + scan['tau']
                knot_t = np.append(np.linspace(t_i.min(), t_i.max(), 5), t_c)
                order = np.argsort(knot_t)
                knot_t = knot_t[order]
                try:
                    poses = [self.pose_at(t) for t in knot_t]
                    c = self.pose_at(t_c)
                except LookupError:
                    if self.now() - stamp < cfg['tf_wait_s']:
                        return None
                    self.skip('tf_missing')
                    c = None
                if c is not None:
                    use, P, P_raw, yaw_rate, e_mc = self.points(scan, t_i, knot_t, poses, c, t_c)
        if self.extrinsic.level == ExtrinsicEstimate.ERROR:
            self.held.clear()
        ctx = Context(cfg, self.tracker, self.K or (1, 1, 0, 0), self.image or (1, 1),
                      getattr(self, 'A', np.eye(3)), e_mc, self.extrinsic.se[1]
                      if np.isfinite(self.extrinsic.se[1]) else 0.0)
        rows, _ = adjust(msg.observations, P, ctx, t_c, self.rates, self.held, use)
        if use:
            self.sample_extrinsic(msg.observations, P_raw, ctx, t_c, yaw_rate, rows)
        out = self.message(msg, rows)
        self.proc.append(time.process_time() - started)
        out.processing_latency = msg.processing_latency + self.proc[-1]
        return out

    def points(self, scan, t_i, knot_t, poses, c, t_c):
        """Gate, compensate and project one scan: (use, P corrected, P raw, yaw rate, e_mc)."""
        cfg = self.cfg
        none = (False, np.zeros((0, 3)), np.zeros((0, 3)), 0.0, 0.0)
        if tf_gap(self.tf_stamps, knot_t[0], knot_t[-1]) > cfg['tf_max_gap_s']:
            self.skip('tf_gap')
            return none
        yaws = np.unwrap([matrix_rpy(R)[2] for R, _ in poses])
        span = knot_t[-1] - knot_t[0]
        yaw_rate = float((yaws[-1] - yaws[0]) / span) if span > 1e-3 else 0.0
        rate = len(self.attitude) / 2.0
        sigma_w = cfg['sigma_omega'] if not cfg['attitude_topic'] or \
            rate >= cfg['attitude_min_hz'] else cfg['sigma_omega_slow']
        e_mc = sigma_w * abs(scan['t_med'] - t_c) + abs(yaw_rate) * cfg['sigma_t_s']
        if e_mc > cfg['e_mc_max_deg'] * DEG:
            self.skip('motion')
            return none
        knots = delta_knots(poses, c)
        pb = compensate(scan['p'], t_i, knot_t, knots)
        near = np.hypot(pb[:, 0], pb[:, 1]) < cfg['water_range_m']
        pb = pb[~(near & (pb[:, 2] < cfg['water_z_m'] + 0.05))]
        R_bo, t_bo = self.camera
        P_raw = (pb - t_bo) @ R_bo           # optical = R_bo^T (p - t)
        P_raw = P_raw[P_raw[:, 2] > 0.1]
        fx, fy, cx, cy = self.K
        u, v = project(P_raw, self.K)
        W, H = self.image
        m = cfg['image_margin_px']
        P_raw = P_raw[(u > -m) & (u < W + m) & (v > -m) & (v < H + m)]
        P = P_raw @ small_rotation(-self.extrinsic.applied).T
        return True, P, P_raw, yaw_rate, e_mc

    def sample_extrinsic(self, obs, P_raw, ctx, t, yaw_rate, rows):
        lidar = {i for s, i, *_ in rows if s in ('lidar', 'split')}
        for b, c in extrinsic_samples(obs, P_raw, ctx):
            if c is not None and self.extrinsic.add(t, b.ob.backend_id, b, c, ctx, yaw_rate):
                self.ex_log.append(self.extrinsic.last)
            self.extrinsic.observe_isolated(t, c is not None, b.index in lidar)
        if self.extrinsic.level == ExtrinsicEstimate.ERROR and self.extrinsic.message:
            self.get_logger().error(self.extrinsic.message, throttle_duration_sec=10.0)

    def message(self, msg, rows):
        from copy import deepcopy
        out = deepcopy(msg)
        out.observations = []
        self.sources = [s for s, *_ in rows]
        for source, i, p, cov, box2d, key in rows:
            self.counts[source] += 1
            ob = deepcopy(msg.observations[i]) if source == 'split' else msg.observations[i]
            if p is not None:
                ob.position.x, ob.position.y, ob.position.z = map(float, p)
                ob.position_covariance = cov
            if box2d is not None:
                ob.box2d = [float(b) for b in box2d]
                ob.score = float(msg.observations[i].score) * 0.9
                ob.backend_id = key
            out.observations.append(ob)
        return out

    def report(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        from socket import gethostname
        from mhseals_nav.perception_status import status
        ex = self.extrinsic
        wall, cpu = time.monotonic(), time.process_time()
        load = (cpu - self.cpu_at[1]) / max(wall - self.cpu_at[0], 1e-6)
        self.cpu_at = (wall, cpu)
        proc = sorted(self.proc)
        boxed = sum(self.counts[s] for s in ('lidar', 'split', 'held', 'stereo'))
        p95 = proc[int(.95 * (len(proc) - 1))] if proc else math.nan
        values = {'cpu_pct': f'{100 * load:.0f}',
                  'proc_ms_p95': f'{1e3 * p95:.1f}',
                  'lidar_share': f'{(self.counts["lidar"] + self.counts["split"]) / boxed:.2f}'
                  if boxed else 'nan',
                  **{f'n_{k}': v for k, v in self.counts.items()},
                  **{f'skip_{k}': v for k, v in sorted(self.skips.items())},
                  'scans': len(self.scans),
                  'extrinsic_n': len(ex.samples),
                  'extrinsic_deg': ' '.join(f'{a:.2f}' for a in np.degrees(ex.theta[:3])),
                  'extrinsic_se_deg': ' '.join(f'{a:.2f}' for a in np.degrees(ex.se[:3])),
                  'applied_deg': ' '.join(f'{a:.2f}' for a in np.degrees(ex.applied)),
                  'c_m': ' '.join(f'{a:.3f}' for a in ex.theta[3:6]),
                  'tau_ms': f'{1e3 * ex.theta[6]:.1f}',
                  'alerts': ','.join(sorted(ex.alerts))}
        level = [DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR][ex.level]
        if ex.alerts and level == DiagnosticStatus.OK:
            level = DiagnosticStatus.WARN
        array = self.Diag()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status('fusion', gethostname(), values, level,
                               ex.message or ','.join(sorted(ex.alerts)), source='lidar')]
        self.status_pub.publish(array)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = LidarFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
