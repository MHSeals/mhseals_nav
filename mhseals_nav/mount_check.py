"""Pre-run sensor mount checks from a short dock bag (0-5 deg unknown mount tilts).

Three absolute checks the online lidar-camera check cannot make, each against the URDF in use:
- taped buoy: the lidar's range and bearing to a buoy taped at a known base_link (x, y). A
  whole-rail move keeps lidar and camera agreeing; only an absolute reference sees it.
- lidar tilt: a robust plane through near-field water returns, in base_link through the URDF
  mount. Water is level, so its normal's tilt minus the FCU's hull roll/pitch is the lidar
  mount's roll/pitch error. Its intercept is the water height (lidar_fusion water_z_m).
- camera tilt: the horizon line in the rectified image gives camera pitch and roll against
  level; minus the hull attitude, the camera mount's error. Horizon dip at 0.4 m is 0.02 deg.
The FCU's /imu/raw attitude has its level trim (AHRS_TRIM_X/Y) applied: the hull attitude here is
the FCU's idea of level. Few returns or no horizon read UNOBSERVABLE, never level.

  python3 -m mhseals_nav.mount_check BAG [--buoy X,Y] [--config PATH]
"""
import argparse
import math
from pathlib import Path
import sys

from mhseals_nav.robot_geometry import description_dir
import numpy as np
import yaml

OK, WARN, ERROR, UNOBSERVABLE = 'OK', 'WARN', 'ERROR', 'UNOBSERVABLE'
CONFIG = description_dir().parent / 'config' / 'mount_check.yaml'


def load_config(path=CONFIG):
    cfg = yaml.safe_load(Path(path).read_text())['mount_check']
    keys = {'topics', 'buoy_radius_m', 'buoy_gate_m', 'range_tol_m', 'bearing_tol_deg',
            'water_range_m', 'water_band_z_m', 'plane_tol_m', 'plane_min_points',
            'plane_min_spread_m', 'tilt_warn_deg', 'tilt_error_deg', 'horizon_min_inliers'}
    if set(cfg) != keys:
        raise KeyError(f'mount_check config keys differ: {sorted(set(cfg) ^ keys)}')
    return cfg


def verdict(error_deg, cfg):
    worst = max(abs(e) for e in error_deg)
    return OK if worst < cfg['tilt_warn_deg'] else WARN if worst < cfg['tilt_error_deg'] else ERROR


def taped_buoy(points_base, taped_xy, cfg):
    """Lidar returns near a taped buoy -> (range error m, bearing error deg, verdict).

    The returns are the buoy's near face: the centre is one radius further along the ray.
    """
    taped = np.asarray(taped_xy, float)
    near = points_base[np.linalg.norm(points_base[:, :2] - taped, axis=1) < cfg['buoy_gate_m']]
    if len(near) < 5:
        return None, None, UNOBSERVABLE
    face = near[:, :2].mean(axis=0)
    centre = face * (1 + cfg['buoy_radius_m'] / np.linalg.norm(face))
    dr = float(np.linalg.norm(centre) - np.linalg.norm(taped))
    db = math.degrees(math.remainder(math.atan2(centre[1], centre[0])
                                     - math.atan2(taped[1], taped[0]), 2 * math.pi))
    ok = abs(dr) < cfg['range_tol_m'] and abs(db) < cfg['bearing_tol_deg']
    return dr, db, OK if ok else ERROR


def fit_plane(points, tol, iterations=200, seed=0):
    """RANSAC then least squares: (unit normal, n_z > 0; offset d, n.p + d = 0; inliers)."""
    rng = np.random.default_rng(seed)
    best = np.zeros(len(points), bool)
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inliers = np.abs((points - a) @ n) < tol
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 3:
        return None, None, best
    p = points[best]
    centre = p.mean(axis=0)
    n = np.linalg.svd(p - centre)[2][-1]
    n = n if n[2] > 0 else -n
    return n, float(-n @ centre), best


def normal_roll_pitch(n):
    """Roll and pitch (deg, REP-103, pitch nose-down positive) of a frame seeing up as n."""
    roll, pitch = math.atan2(n[1], n[2]), math.atan2(-n[0], math.hypot(n[1], n[2]))
    return math.degrees(roll), math.degrees(pitch)


def water_tilt(points_base, cfg):
    """One scan -> (roll, pitch of the water seen in base_link deg, water z m) or None."""
    r = np.linalg.norm(points_base[:, :2], axis=1)
    lo, hi = cfg['water_range_m']
    zlo, zhi = cfg['water_band_z_m']
    z = points_base[:, 2]
    band = points_base[(r > lo) & (r < hi) & (z > zlo) & (z < zhi)]
    if len(band) < cfg['plane_min_points']:
        return None
    n, d, inliers = fit_plane(band, cfg['plane_tol_m'])
    if n is None or inliers.sum() < cfg['plane_min_points']:
        return None
    spread = np.sqrt(np.linalg.eigvalsh(np.cov(band[inliers][:, :2].T)))
    if spread.min() < cfg['plane_min_spread_m']:   # one strip or ring: tilt unobservable
        return None
    return (*normal_roll_pitch(n), -d / n[2])


def horizon(gray, fx, fy, cx, cy, cfg, seed=0):
    """Rectified grey image -> (camera pitch, roll deg against the horizon) or None."""
    # Per column, the strongest vertical edge; a RANSAC line through them.
    g = np.abs(np.diff(gray.astype(np.float64), axis=0))
    rows = g.argmax(axis=0).astype(np.float64) + 0.5
    cols = np.arange(len(rows), dtype=np.float64)
    rng = np.random.default_rng(seed)
    best = np.zeros(len(rows), bool)
    for _ in range(200):
        i, j = rng.choice(len(rows), 2, replace=False)
        if cols[i] == cols[j]:
            continue
        slope = (rows[j] - rows[i]) / (cols[j] - cols[i])
        inliers = np.abs(rows - (rows[i] + slope * (cols - cols[i]))) < 2.0
        if inliers.sum() > best.sum():
            best = inliers
    if best.mean() < cfg['horizon_min_inliers']:
        return None
    slope, row0 = np.polyfit(cols[best], rows[best], 1)
    v = row0 + slope * cx
    return math.degrees(math.atan2(cy - v, fy)), math.degrees(math.atan(-slope * fy / fx))


def hull_attitude(q):
    """FCU attitude quaternion (x, y, z, w) -> (roll, pitch) deg."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1., min(1., 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def read_bag(bag, topics):
    """{topic: [(t, message)]} for the named topics."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t = reader.read_next()
        out[topic].append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag')
    parser.add_argument('--buoy', help='taped buoy centre in base_link, X,Y m')
    parser.add_argument('--config', default=CONFIG)
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from mhseals_nav.robot_geometry import RobotGeometry
    topics = cfg['topics']
    msgs = read_bag(args.bag, topics.values())
    geometry = RobotGeometry.load()
    hull = [hull_attitude((m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w))
            for _, m in msgs[topics['imu']]]
    hull = tuple(np.median(np.array(hull), axis=0)) if hull else None
    print('hull attitude (FCU, level trim applied): '
          + (f'roll {hull[0]:+.2f} pitch {hull[1]:+.2f} deg' if hull else 'no /imu data'))
    failed = False

    scans = []
    for _, m in msgs[topics['points']]:
        R, t = geometry.pose(m.header.frame_id)
        scans.append(read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True) @ R.T + t)
    if args.buoy:
        x, y = (float(v) for v in args.buoy.split(','))
        results = [taped_buoy(s, (x, y), cfg) for s in scans]
        good = [(dr, db) for dr, db, v in results if v != UNOBSERVABLE]
        if good:
            dr, db = np.median(np.array(good), axis=0)
            ok = abs(dr) < cfg['range_tol_m'] and abs(db) < cfg['bearing_tol_deg']
            print(f'taped buoy: range {dr:+.3f} m bearing {db:+.2f} deg over {len(good)} scans '
                  f'-> {OK if ok else ERROR}')
            failed |= not ok
        else:
            print(f'taped buoy: no returns within {cfg["buoy_gate_m"]} m -> {UNOBSERVABLE}')

    fits = [f for f in (water_tilt(s, cfg) for s in scans) if f]
    if fits and hull:
        roll, pitch, z = np.median(np.array(fits), axis=0)
        spread = np.ptp(np.array(fits)[:, :2], axis=0).max()
        error = (roll - hull[0], pitch - hull[1])
        v = verdict(error, cfg)
        print(f'lidar: water roll {roll:+.2f} pitch {pitch:+.2f} deg (scan spread {spread:.2f}), '
              f'mount error roll {error[0]:+.2f} pitch {error[1]:+.2f} deg, water z {z:+.3f} m '
              f'over {len(fits)}/{len(scans)} scans -> {v}')
        failed |= v == ERROR
    else:
        print(f'lidar: no level water plane in {len(scans)} scans -> {UNOBSERVABLE}')

    infos = msgs[topics['camera_info']]
    estimates = []
    for _, m in msgs[topics['image']][::10] if infos else []:
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(m.encoding)
        if channels is None:
            continue
        image = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * channels]
        gray = image.reshape(m.height, m.width, channels)[:, :, :3].mean(axis=2)
        k = infos[-1][1].k
        estimate = horizon(gray, k[0], k[4], k[2], k[5], cfg)
        if estimate:
            estimates.append(estimate)
    if estimates and hull:
        pitch, roll = np.median(np.array(estimates), axis=0)
        urdf = normal_roll_pitch(geometry.pose('front_camera_link')[0].T[:, 2])
        error = (roll - hull[0] - urdf[0], pitch - hull[1] - urdf[1])
        v = verdict(error, cfg)
        print(f'camera: horizon pitch {pitch:+.2f} roll {roll:+.2f} deg, mount error roll '
              f'{error[0]:+.2f} pitch {error[1]:+.2f} deg over {len(estimates)} frames -> {v}')
        failed |= v == ERROR
    else:
        print(f'camera: no horizon -> {UNOBSERVABLE}')
    print('a mount error goes into blastoise.xacro lidar_rpy / camera_rpy (roll, pitch in rad)')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
