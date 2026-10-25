"""Per-frame camera attitude and single-camera (waterline) range. Pure: no ROS I/O.

Attitude: the camera IMU's roll and pitch, interpolated at the capture stamp, plus a slowly
adapted bias toward the horizon fixes (horizon.detect). The horizon is absolute but slow and
sometimes absent; the IMU is fast but its gravity reference leans in turns and surges and
carries a mounting offset. Without a fresh fix the IMU alone is used, with a wider sigma.

Waterline range: the ray through the bottom centre of a detection's 2D box meets the water
plane ``height`` below the camera. Its error grows as rho^2 / height times the angle error, so
the sigma is honest about range, attitude and box-edge noise; the tracker weighs it against
stereo depth.
"""

from collections import deque
import math

from mhseals_nav.horizon import ray, rotation
import numpy as np


def mount_matrix(rpy_deg):
    """Return the fixed rotation sensor <- camera from roll, pitch, yaw in degrees (ZYX)."""
    r, p, y = (math.radians(a) for a in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), \
        math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def camera_roll_pitch(q_imu, mount):
    """IMU orientation (world <- imu) and mount (imu <- camera) -> camera roll, pitch."""
    R = quat_matrix(q_imu) @ mount
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return roll, pitch


class Attitude:
    """Camera roll and pitch at a stamp from IMU samples and horizon fixes.

    cfg keys: imu_sigma_deg (IMU alone), horizon_sigma_deg (floor of a fused estimate),
    bias_tau_s (horizon -> IMU bias adaptation), fresh_s (a fix this old still counts),
    max_gap_s (IMU interpolation gap), buffer_s.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.imu = deque()          # (t, roll, pitch)
        self.bias = np.zeros(2)
        self.bias_t = None
        self.fix = None             # (t, roll, pitch, sigma)
        self.fixes = 0

    def add_imu(self, t, roll, pitch):
        if self.imu and t <= self.imu[-1][0]:
            return
        self.imu.append((t, roll, pitch))
        while self.imu and t - self.imu[0][0] > self.cfg['buffer_s']:
            self.imu.popleft()

    def imu_at(self, t):
        """Interpolated IMU (roll, pitch) at t, or None outside the buffer or across a gap."""
        h = self.imu
        if not h or t < h[0][0] or t > h[-1][0] + self.cfg['max_gap_s']:
            return None
        if t >= h[-1][0]:
            return np.array(h[-1][1:])
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (ta, ra, pa), (tb, rb, pb) = h[lo], h[hi]
        if tb - ta > self.cfg['max_gap_s']:
            return None
        f = (t - ta) / (tb - ta) if tb > ta else 0.0
        return np.array([ra + f * (rb - ra), pa + f * (pb - pa)])

    def add_horizon(self, t, roll, pitch, sigma):
        self.fix = (t, roll, pitch, sigma)
        self.fixes += 1
        imu = self.imu_at(t)
        if imu is None:
            return
        err = np.array([roll, pitch]) - imu
        if self.bias_t is None:
            self.bias, self.bias_t = err, t
            return
        gain = 1 - math.exp(-max(t - self.bias_t, 0.0) / self.cfg['bias_tau_s'])
        self.bias = self.bias + gain * (err - self.bias)
        self.bias_t = t

    def at(self, t):
        """(roll, pitch, sigma, source) at t; source 'horizon', 'imu' or None."""
        cfg = self.cfg
        imu = self.imu_at(t)
        fresh = self.fix is not None and abs(t - self.fix[0]) <= cfg['fresh_s']
        horizon_sigma = math.radians(cfg['horizon_sigma_deg'])
        if imu is not None:
            if self.bias_t is not None and abs(t - self.bias_t) <= cfg['fresh_s']:
                r, p = imu + self.bias
                return r, p, horizon_sigma, 'horizon'
            r, p = imu + (self.bias if self.bias_t is not None else 0.0)
            return r, p, math.radians(cfg['imu_sigma_deg']), 'imu'
        if fresh:
            return self.fix[1], self.fix[2], max(horizon_sigma, self.fix[3]), 'horizon'
        return None


def fill_mono(observations, K, image_size, estimate, cfg):
    """Set mono_range and mono_range_sigma of each observation with a 2D box; returns count.

    ``image_size``: (width, height) K refers to; boxes are normalized to it. ``estimate``:
    Attitude.at() result or None (then nothing is filled).
    """
    if estimate is None or not image_size:
        return 0
    roll, pitch, sigma, _ = estimate
    width, height = image_size
    filled = 0
    for ob in observations:
        if not ob.has_box2d:
            continue
        u0, v0, u1, v1 = ob.box2d
        if v1 >= 0.999:          # cut by the bottom edge: the waterline is not in view
            continue
        half = min(ob.size.x, ob.size.y) / 2 if ob.size.x > 0 and ob.size.y > 0 else 0.0
        found = mono_range((u0 * width, v0 * height, u1 * width, v1 * height), K, roll, pitch,
                           sigma, cfg, half_depth=min(half, 1.0))
        if found is not None:
            ob.mono_range, ob.mono_range_sigma = found
            filled += 1
    return filled


def mono_range(box_px, K, roll, pitch, att_sigma, cfg, half_depth=0.0):
    """(range, sigma) of the object centre on the water plane, or None.

    ``box_px``: u0 v0 u1 v1 in K's pixels. ``range`` is horizontal, from the camera origin.
    cfg keys: camera_height_m, height_sigma_m, edge_sigma_px, min_depression_deg,
    max_range_m, centre_sigma_m.
    """
    u0, v0, u1, v1 = box_px
    fx, fy, cx, cy = K
    u = 0.5 * (u0 + u1)
    d = rotation(roll, pitch) @ ray(u, v1, K)
    horizontal = math.hypot(d[0], d[1])
    depression = math.atan2(-d[2], horizontal)
    if depression < math.radians(cfg['min_depression_deg']):
        return None
    h = cfg['camera_height_m']
    contact = h / math.tan(depression)
    rng = contact + half_depth
    if rng > cfg['max_range_m']:
        return None
    # Angle error: box edge (pixels), pitch, and roll times the column's lateral slope.
    s_alpha = math.hypot(math.hypot(cfg['edge_sigma_px'] / fy, att_sigma),
                         att_sigma * abs(u - cx) / fx)
    s_angle = h / math.sin(depression) ** 2 * s_alpha
    s_height = contact * cfg['height_sigma_m'] / h
    sigma = math.sqrt(s_angle ** 2 + s_height ** 2 + cfg['centre_sigma_m'] ** 2)
    return rng, sigma
