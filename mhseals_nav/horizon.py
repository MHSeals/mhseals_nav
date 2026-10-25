"""Horizon line -> camera roll and pitch, numpy only (no ROS, no OpenCV).

Camera axes follow ROS (x forward, y left, z up); pixels follow the rectified image whose
intrinsics are K (fx, fy, cx, cy). The camera's attitude is R = Ry(pitch) Rx(roll): pitch is
positive nose down, roll positive left side up, as in a ZYX (yaw-pitch-roll) quaternion.

detect() looks for the longest straight, near-horizontal intensity edge (the sky or shore over
water) on a block-averaged copy of the frame, optionally inside a band around the horizon a
prior attitude predicts, and fits it with a deterministic RANSAC. Each column gives at most one
candidate (its strongest vertical gradient in the band), so short edges of buoys, boats and
waves cannot outvote a line that spans the frame.
"""

import math

import numpy as np


def ray(u, v, K):
    """Pixel -> unnormalized ray in camera axes (x forward)."""
    fx, fy, cx, cy = K
    return np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])


def up_vector(roll, pitch):
    """World up in camera axes for R = Ry(pitch) Rx(roll)."""
    return np.array([-math.sin(pitch), math.sin(roll) * math.cos(pitch),
                     math.cos(roll) * math.cos(pitch)])


def rotation(roll, pitch):
    """R = Ry(pitch) Rx(roll): camera axes -> level axes (yaw left out)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx


def horizon_row(u, roll, pitch, K, dip=0.0):
    """Image row of the horizon at column(s) u; ``dip`` (rad) lowers it, as a shore does."""
    fx, fy, cx, cy = K
    u = np.asarray(u, dtype=float)
    # n . ray(u, v) = 0, with the up vector tilted by the dip.
    n = up_vector(roll, pitch - dip)
    return cy + fy * (n[0] - n[1] * (u - cx) / fx) / n[2]


def attitude_from_line(p0, p1, K, dip=0.0):
    """Two image points on the horizon -> (roll, pitch) in rad."""
    n = np.cross(ray(*p0, K), ray(*p1, K))
    if n[2] < 0:
        n = -n
    n /= np.linalg.norm(n)
    pitch = math.asin(max(-1.0, min(1.0, -n[0]))) + dip
    roll = math.atan2(n[1], n[2])
    return roll, pitch


def gray(image):
    """Return one channel of a colour frame (green stands in for luma), or a grey frame."""
    a = np.asarray(image)
    return a[..., 1] if a.ndim == 3 else a


def downscale(a, s):
    """Block mean by an integer factor ``s`` (edges cropped), as float32."""
    if s <= 1:
        return a.astype(np.float32)
    h, w = (a.shape[0] // s) * s, (a.shape[1] // s) * s
    return a[:h, :w].reshape(h // s, s, w // s, s).mean(axis=(1, 3), dtype=np.float32)


def detect(image, K, cfg, prior=None):
    """Return the horizon fix as a dict, or None.

    ``image``: frame matching K (HxW or HxWxC). ``prior``: (roll, pitch) to search a band
    of +-``band_deg`` around, or None for the whole frame. Result keys: roll, pitch (rad),
    sigma (rad, residual spread as an angle), inliers (share of columns), line (u0 v0 u1 v1
    full-resolution pixels).
    """
    s = int(cfg.get('scale', 4))
    img = downscale(gray(image), s)
    h, w = img.shape
    if h < 8 or w < 16:
        return None
    fx, fy, cx, cy = K
    Ks = (fx / s, fy / s, (cx + 0.5) / s - 0.5, (cy + 0.5) / s - 0.5)
    # 3x3 box blur, then central differences.
    pad = np.pad(img, 1, mode='edge')
    blur = sum(pad[i:i + h, j:j + w] for i in range(3) for j in range(3)) / 9.0
    gy = np.zeros_like(blur)
    gy[1:-1] = (blur[2:] - blur[:-2]) / 2
    gx = np.zeros_like(blur)
    gx[:, 1:-1] = (blur[:, 2:] - blur[:, :-2]) / 2
    strength = np.abs(gy)
    # Near-horizontal edges only (|gx| small against |gy|); band rows and margins.
    strength[np.abs(gx) > cfg.get('max_slope', 1.0) * np.abs(gy)] = 0.0
    margin = 2
    strength[:margin], strength[-margin:] = 0.0, 0.0
    cols = np.arange(w)
    if prior is not None:
        centre = horizon_row(cols, prior[0], prior[1], Ks)
        band = Ks[1] * math.tan(math.radians(cfg.get('band_deg', 6.0)))
        rows = np.arange(h)[:, None]
        strength[np.abs(rows - centre[None, :]) > band] = 0.0
    # Per column, the lowest edge at least half as strong as the column's strongest: the
    # water meets the shore (or sky) below a tree line, never above it.
    colmax = strength.max(axis=0)
    strong = strength >= np.maximum(cfg.get('min_grad', 3.0), 0.5 * colmax)[None, :]
    best = h - 1 - np.argmax(strong[::-1], axis=0)
    ok = strong[best, cols]
    for _ in range(3):     # climb from the lower flank to that edge's own peak
        up = (best > 0) & (strength[np.maximum(best - 1, 0), cols] > strength[best, cols])
        best = np.where(up, best - 1, best)
    if ok.sum() < cfg.get('min_columns', 0.4) * w:
        return None
    # Sub-row refinement (parabola through the peak and its neighbours).
    r = best.astype(float)
    inner = ok & (best > 0) & (best < h - 1)
    a, b, c = (strength[np.clip(best + k, 0, h - 1), cols] for k in (-1, 0, 1))
    den = a - 2 * b + c
    with np.errstate(divide='ignore', invalid='ignore'):
        shift = np.where(inner & (den < 0), 0.5 * (a - c) / den, 0.0)
    r = r + np.clip(shift, -0.5, 0.5)
    xs, ys = cols[ok].astype(float), r[ok]
    tol = cfg.get('inlier_px', 1.0)
    rng = np.random.default_rng(0)
    best_in = None
    for _ in range(int(cfg.get('iterations', 60))):
        i, j = rng.choice(len(xs), 2, replace=False)
        if xs[i] == xs[j]:
            continue
        slope = (ys[j] - ys[i]) / (xs[j] - xs[i])
        inl = np.abs(ys - (ys[i] + slope * (xs - xs[i]))) <= tol
        if best_in is None or inl.sum() > best_in.sum():
            best_in = inl
    if best_in is None or best_in.sum() < cfg.get('min_inliers', 0.5) * w:
        return None
    xi, yi = xs[best_in], ys[best_in]
    if xi.max() - xi.min() < cfg.get('min_span', 0.6) * w:
        return None
    slope, icpt = np.polyfit(xi, yi, 1)
    resid = yi - (slope * xi + icpt)
    # Back to full-resolution pixel centres.
    u0, u1 = 0.0, float(w * s - 1)
    full = lambda u: ((slope * ((u + 0.5) / s - 0.5) + icpt) + 0.5) * s - 0.5  # noqa: E731
    v0, v1 = full(u0), full(u1)
    dip = math.radians(cfg.get('dip_deg', 0.0))
    roll, pitch = attitude_from_line((u0, v0), (u1, v1), K, dip)
    sigma = float(np.std(resid)) * s / fy / math.sqrt(max(len(xi) / 4.0, 1.0))
    return {'roll': roll, 'pitch': pitch, 'sigma': sigma,
            'inliers': float(best_in.sum()) / w, 'line': (u0, v0, u1, v1)}
