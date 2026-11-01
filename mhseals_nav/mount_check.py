"""Pre-run sensor mount checks from a short dock bag (0-5 deg unknown mount tilts).

Three absolute checks the online lidar-camera check cannot make, each against the mounts in use
(the bag's /tf_static when recorded, else the repo URDF):
- taped buoy: the lidar's range and bearing to a buoy taped at a known base_link (x, y). A
  whole-rail move keeps lidar and camera agreeing; only an absolute reference sees it.
- lidar tilt: a robust plane through near-field water returns, in base_link through the mount.
  Water is level, so its normal's tilt minus the FCU's hull roll/pitch is the lidar mount's
  roll/pitch error. Its intercept is the water height (lidar_fusion water_z_m). A dock deck is
  denser than calm water, so a plane counts as water only at the waterline the hull allows,
  with returns all around the boat, and the lowest such plane wins.
- camera tilt: the water edge in the rectified image (the lowest strong edge with water under
  it, not the treeline above it) gives camera pitch and roll against level; minus the hull
  attitude, the camera mount's error. Horizon dip at 0.4 m is 0.0003 rad.
The FCU's /imu/raw attitude has its level trim (AHRS_TRIM_X/Y) applied: the hull attitude here is
the FCU's idea of level, taken per scan or frame at its stamp. Few returns, a plane that may be a
deck or an ambiguous edge read UNOBSERVABLE, never level. Angles in rad.

  python3 -m mhseals_nav.mount_check BAG [--buoy X,Y] [--config PATH]

Exit 0 when every observed check passes, 1 on an ERROR, 3 when no check was observable.
"""
import argparse
import math
from pathlib import Path
import sys

from mhseals_nav.navigation_config import load_strict
from mhseals_nav.robot_geometry import description_dir, quat_matrix, RobotGeometry
import numpy as np

OK, WARN, ERROR, UNOBSERVABLE = 'OK', 'WARN', 'ERROR', 'UNOBSERVABLE'
EXIT_UNOBSERVABLE = 3
CONFIG = description_dir().parent / 'config' / 'mount_check.yaml'
TOPIC, POS, NUM, INT, PAIR = 'topic', 'pos', 'num', 'int', 'pair'
SCHEMA = {
    'topics': {key: (TOPIC,) for key in ('points', 'imu', 'image', 'camera_info', 'tf_static')},
    'attitude_max_dt': (POS, 1.0),
    'buoy': {'radius': (POS, 2.0), 'gate': (POS, 5.0), 'cluster_gap': (POS, 1.0),
             'fit_tolerance': (POS, 0.5), 'min_points': (INT, 3, 10000),
             'range_tolerance': (POS, 2.0), 'bearing_tolerance': (POS, 0.5)},
    'water': {'range': (PAIR, 0.0, 100.0), 'band': (PAIR, -10.0, 10.0),
              'waterline': (NUM, -5.0, 5.0, 'nullable'), 'waterline_band': (POS, 1.0),
              'waterline_range': (PAIR, -5.0, 5.0), 'max_tilt': (POS, 0.5),
              'plane_tolerance': (POS, 1.0), 'min_points': (INT, 3, 100000),
              'min_spread': (POS, 10.0), 'candidates': (INT, 1, 10), 'sectors': (INT, 4, 72),
              'min_sectors': (INT, 1, 72), 'sector_min_points': (INT, 1, 10000)},
    'horizon': {'min_inliers': (POS, 1.0), 'edge_min': (POS, 255.0),
                'texture_rows': (INT, 1, 100), 'texture_max': (POS, 255.0),
                'shore_elevation': (NUM, -0.2, 0.2), 'mirror_span': (POS, 0.5),
                'mirror_correlation': (PAIR, -1.0, 1.0)},
    'verdict': {'tilt_warn': (POS, 0.5), 'tilt_error': (POS, 0.5), 'spread_warn': (POS, 0.5)},
}


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _validate(data, schema, path):
    if not isinstance(data, dict) or set(data) != set(schema):
        found = set(data) if isinstance(data, dict) else set()
        raise ValueError(f'{path}: keys differ: {sorted(found ^ set(schema))}')
    for key, spec in schema.items():
        value, name = data[key], f'{path}.{key}'
        if isinstance(spec, dict):
            _validate(value, spec, name)
        elif spec[0] == TOPIC:
            if not isinstance(value, str) or not value.startswith('/') or ' ' in value:
                raise ValueError(f'{name} must be an absolute topic name')
        elif spec[0] == INT:
            if type(value) is not int or not spec[1] <= value <= spec[2]:
                raise ValueError(f'{name} must be an integer in {spec[1]}..{spec[2]}')
        elif spec[0] == PAIR:
            if (not isinstance(value, list) or len(value) != 2 or not all(map(_number, value))
                    or not spec[1] <= value[0] < value[1] <= spec[2]):
                raise ValueError(f'{name} must be [low, high] within {spec[1]}..{spec[2]}')
        elif spec[0] == POS:
            if not _number(value) or not 0 < value <= spec[1]:
                raise ValueError(f'{name} must be a number in (0, {spec[1]}]')
        elif value is None and spec[3:] == ('nullable',):
            continue
        elif not _number(value) or not spec[1] <= value <= spec[2]:
            raise ValueError(f'{name} must be a number in {spec[1]}..{spec[2]}')


def load_config(path=CONFIG):
    """Strict-load mount_check.yaml: exact nested keys, types and ranges."""
    data = load_strict(Path(path).read_bytes(), path)
    if not isinstance(data, dict) or set(data) != {'mount_check'}:
        raise ValueError(f'{path}: expected a single mount_check mapping')
    cfg = data['mount_check']
    _validate(cfg, SCHEMA, 'mount_check')
    if cfg['water']['min_sectors'] > cfg['water']['sectors']:
        raise ValueError('mount_check.water.min_sectors exceeds sectors')
    if cfg['verdict']['tilt_warn'] >= cfg['verdict']['tilt_error']:
        raise ValueError('mount_check.verdict.tilt_warn must be below tilt_error')
    return cfg


def verdict(errors, cfg):
    """Per-scan mount errors [(roll, pitch)] -> (median error, scan spread, verdict)."""
    errors = np.asarray(errors, float)
    error, spread = np.median(errors, axis=0), float(errors.std(axis=0).max())
    worst = np.abs(error).max()
    v = OK if worst < cfg['tilt_warn'] else WARN if worst < cfg['tilt_error'] else ERROR
    if v == OK and spread > cfg['spread_warn']:
        v = WARN
    return tuple(error), spread, v


def clusters(xy, gap):
    """Single-linkage clusters of 2-D points (index arrays): neighbours closer than gap."""
    label = np.full(len(xy), -1)
    for seed in range(len(xy)):
        if label[seed] >= 0:
            continue
        label[seed], frontier = seed, [seed]
        while frontier:
            i = frontier.pop()
            near = np.flatnonzero((label < 0) & (np.linalg.norm(xy - xy[i], axis=1) < gap))
            label[near] = seed
            frontier.extend(near)
    return [np.flatnonzero(label == k) for k in np.unique(label)]


def fit_circle(xy, radius, iterations=20):
    """Centre of a circle of known radius through a near-face arc -> (centre, rms) or None.

    Starts from the arc correction (a half circle's centroid is 2r/pi = 0.64r nearer than its
    centre), then Gauss-Newton on the centre. A centre nearer than the returns is no buoy.
    """
    face = xy.mean(axis=0)
    ray = face / np.linalg.norm(face)
    c = face + 2 * radius / math.pi * ray
    for _ in range(iterations):
        d = xy - c
        dist = np.maximum(np.linalg.norm(d, axis=1), 1e-9)
        step = np.linalg.lstsq(-d / dist[:, None], -(dist - radius), rcond=None)[0]
        c = c + step
        if np.linalg.norm(step) < 1e-6:
            break
    if (c - face) @ ray <= 0:
        return None
    return c, float(np.sqrt(np.mean((np.linalg.norm(xy - c, axis=1) - radius) ** 2)))


def taped_buoy(points_base, taped_xy, cfg):
    """Lidar returns near a taped buoy -> (range error m, bearing error rad, verdict).

    Each cluster near the taped centre is fitted with a circle of the buoy's radius; one that
    fits, and whose returns span between one radius and the diameter across the ray, is the
    buoy. Docks and poles fail the fit or the width; none or several read UNOBSERVABLE.
    """
    taped, r = np.asarray(taped_xy, float), cfg['radius']
    xy = points_base[:, :2]
    xy = xy[np.linalg.norm(xy - taped, axis=1) < cfg['gate']]
    found = []
    for index in clusters(xy, cfg['cluster_gap']) if len(xy) else []:
        arc = xy[index]
        if len(arc) < cfg['min_points']:
            continue
        ray = arc.mean(axis=0) / np.linalg.norm(arc.mean(axis=0))
        width = np.ptp(arc @ np.array([-ray[1], ray[0]]))
        fit = fit_circle(arc, r)
        if (fit and fit[1] < cfg['fit_tolerance']
                and r <= width <= 2 * (r + cfg['fit_tolerance'])):
            found.append(fit[0])
    if len(found) != 1:
        return None, None, UNOBSERVABLE
    centre = found[0]
    dr = float(np.linalg.norm(centre) - np.linalg.norm(taped))
    db = math.remainder(math.atan2(centre[1], centre[0]) - math.atan2(taped[1], taped[0]),
                        2 * math.pi)
    ok = abs(dr) < cfg['range_tolerance'] and abs(db) < cfg['bearing_tolerance']
    return dr, db, OK if ok else ERROR


def fit_plane(points, tol, max_tilt=math.pi / 2, iterations=200, seed=0):
    """RANSAC then least squares: (unit normal, n_z > 0; offset d, n.p + d = 0; inliers).

    Only planes within max_tilt of level are tried.
    """
    rng = np.random.default_rng(seed)
    best = np.zeros(len(points), bool)
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if math.acos(min(1.0, abs(n[2]))) > max_tilt:
            continue
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
    """Roll and pitch (rad, REP-103, pitch nose-down positive) of a frame seeing up as n."""
    return math.atan2(n[1], n[2]), math.atan2(-n[0], math.hypot(n[1], n[2]))


def around_the_boat(points, cfg):
    """Check for returns in min_sectors azimuth sectors or more, not all in one half circle."""
    sectors = cfg['sectors']
    index = ((np.arctan2(points[:, 1], points[:, 0]) + math.pi) / (2 * math.pi) * sectors)
    counts = np.bincount(index.astype(int) % sectors, minlength=sectors)
    occupied = counts >= cfg['sector_min_points']
    if occupied.sum() < cfg['min_sectors']:
        return False
    run = longest = 0
    for empty in np.concatenate([~occupied, ~occupied]):
        run = run + 1 if empty else 0
        longest = max(longest, run)
    return longest < sectors / 2


def water_tilt(points_base, cfg):
    """One scan -> (roll, pitch of the water seen in base_link rad, water z m) or None.

    Up to ``candidates`` level-ish planes are peeled off, densest first. A plane is water when
    its height at the boat is within the waterline (the taped one +- waterline_band, else the
    hull's waterline_range), its inliers spread in both horizontal axes and lie all around the
    boat; the lowest such plane wins (a deck floats above the water).
    """
    taped = cfg['waterline']
    wlo, whi = (cfg['waterline_range'] if taped is None
                else (taped - cfg['waterline_band'], taped + cfg['waterline_band']))
    r = np.linalg.norm(points_base[:, :2], axis=1)
    lo, hi = cfg['range']
    zlo, zhi = cfg['band']
    z = points_base[:, 2]
    rest = points_base[(r > lo) & (r < hi) & (z > zlo) & (z < zhi)]
    water = []
    for _ in range(cfg['candidates']):
        if len(rest) < cfg['min_points']:
            break
        n, d, inliers = fit_plane(rest, cfg['plane_tolerance'], cfg['max_tilt'])
        if n is None or inliers.sum() < cfg['min_points']:
            break
        plane, rest = rest[inliers], rest[~inliers]
        height = -d / n[2]
        spread = np.sqrt(np.linalg.eigvalsh(np.cov(plane[:, :2].T)))
        if (wlo <= height <= whi
                and spread.min() >= cfg['min_spread'] and around_the_boat(plane, cfg)):
            water.append((height, *normal_roll_pitch(n)))
    if not water:
        return None
    height, roll, pitch = min(water)
    return roll, pitch, height


def horizon(gray, fx, fy, cx, cy, cfg, seed=0):
    """Rectified grey image -> (camera pitch, roll rad against level) or None.

    Per column, the lowest strong vertical edge with water-like (smooth) rows under it: the
    water's far edge, not the treeline or sky above it. A RANSAC line through those; too few
    columns on one line (choppy water, clutter) is ambiguous.

    Glassy water mirrors the treeline, and the bottom of the reflection is such an edge too, a
    treeline height below the water edge. So the highest strong edge within mirror_span above
    it is found too: when the band between them is symmetric about their per-column midline
    (its flipped correlation, best within 2 px of it, at or above mirror_correlation's high
    end), that axis is the water edge; between the two ends the pairing is ambiguous. The
    edge's known elevation (shore_elevation) is taken off the pitch.
    """
    image = gray.astype(np.float64)
    g = np.abs(np.diff(image, axis=0))
    k = cfg['texture_rows']
    usable = g.shape[0] - k
    if usable < 1:
        return None
    total = np.vstack([np.zeros((1, g.shape[1])), np.cumsum(g, axis=0)])
    below = (total[k + 1:] - total[1:-k]) / k     # mean step over the k rows under each edge
    edge = (g[:usable] > cfg['edge_min']) & (below < cfg['texture_max'])
    cols = np.arange(g.shape[1], dtype=np.float64)
    rng = np.random.default_rng(seed)
    low_seen = edge.any(axis=0)
    low_rows = usable - 1 - np.argmax(edge[::-1], axis=0) + 0.5
    line = _ransac_line(cols, low_rows, low_seen, cfg['min_inliers'], rng)
    if line is None:
        return None
    # The highest strong edge per column between mirror_span and 4 px above the low line; the
    # per-column midpoints cancel the opposite curvature of the two (constant elevation) edges.
    rows = np.arange(g.shape[0])[:, None] + 0.5
    fitted = line[1] + line[0] * cols
    window = ((g > cfg['edge_min']) & (rows < fitted - 4)
              & (rows > fitted - cfg['mirror_span'] * fy))
    high_rows = np.argmax(window, axis=0) + 0.5
    mid = _ransac_line(cols, (low_rows + high_rows) / 2, low_seen & window.any(axis=0),
                       cfg['min_inliers'], rng)
    if mid is not None:
        weak, strong = cfg['mirror_correlation']
        half = np.median((low_rows - high_rows)[low_seen & window.any(axis=0)]) / 2
        symmetry, shift = max((_mirror_correlation(image, (mid[0], mid[1] + shift), half), shift)
                              for shift in np.arange(-2.0, 2.5, 0.5))
        if symmetry >= strong:
            line = (mid[0], mid[1] + shift)
        elif symmetry > weak:
            return None
    slope, row0 = line
    v = row0 + slope * cx
    return math.atan2(cy - v, fy) - cfg['shore_elevation'], math.atan(-slope * fy / fx)


def _ransac_line(cols, rows, seen, min_inliers, rng):
    """(slope, row0) of the RANSAC line through rows[seen], or None under min_inliers of all."""
    total, cols, rows = len(cols), cols[seen], rows[seen]
    if len(rows) < 2:
        return None
    best = np.zeros(len(rows), bool)
    for _ in range(200):
        i, j = rng.choice(len(rows), 2, replace=False)
        slope = (rows[j] - rows[i]) / (cols[j] - cols[i])
        inliers = np.abs(rows - (rows[i] + slope * (cols - cols[i]))) < 2.0
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < min_inliers * total:
        return None
    return tuple(np.polyfit(cols[best], rows[best], 1))


def _mirror_correlation(image, mid, half):
    """Correlation of the rows above the midline (slope, row0) with their mirror below it."""
    h, w = image.shape
    cols = np.arange(w)
    axis = mid[1] + mid[0] * cols + 0.5    # pixel i spans i .. i + 1 (centres at i + 0.5)
    above, mirrored = [], []
    # Clear of the edges for a 2 px shift, and of the rows next to the axis (a smooth image
    # matches itself there).
    for d in np.arange(3.5, half - 3.0):
        top, bottom = np.floor(axis - d).astype(int), np.floor(axis + d).astype(int)
        ok = (top >= 0) & (bottom < h)
        above.append(image[top[ok], cols[ok]])
        mirrored.append(image[bottom[ok], cols[ok]])
    if not above:
        return 0.0
    above, mirrored = np.concatenate(above), np.concatenate(mirrored)
    if above.std() == 0 or mirrored.std() == 0:
        return 0.0
    return float(np.corrcoef(above, mirrored)[0, 1])


def hull_attitude(q):
    """FCU attitude quaternion (x, y, z, w) -> (roll, pitch) rad."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1., min(1., 2 * (w * y - z * x))))
    return roll, pitch


def attitude_at(stamps, attitudes, t, max_dt):
    """Return the FCU attitude nearest stamp t (sorted stamps), or None beyond max_dt."""
    i = int(np.searchsorted(stamps, t))
    near = [j for j in (i - 1, i) if 0 <= j < len(stamps)]
    j = min(near, key=lambda j: abs(stamps[j] - t), default=None)
    return attitudes[j] if j is not None and abs(stamps[j] - t) <= max_dt else None


def mounts(tf_static, frames):
    """(geometry, source): the bag's /tf_static when it reaches every frame, else the URDF."""
    transforms = [(t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'),
                   quat_matrix(t.transform.rotation.x, t.transform.rotation.y,
                               t.transform.rotation.z, t.transform.rotation.w),
                   (t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z))
                  for _, m in tf_static for t in m.transforms]
    why = 'no /tf_static in the bag'
    if transforms:
        geometry = RobotGeometry.from_transforms(transforms)
        try:
            for frame in frames:
                geometry.pose(frame)
            return geometry, "the bag's /tf_static"
        except (KeyError, ValueError) as error:
            why = f"the bag's /tf_static lacks {error}"
    return RobotGeometry.load(), f'the repo URDF ({why})'


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


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def angle(value):
    return f'{value:+.4f} rad ({math.degrees(value):+.2f} deg)'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag')
    parser.add_argument('--buoy', help='taped buoy centre in base_link, X,Y m')
    parser.add_argument('--config', default=CONFIG)
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    topics = cfg['topics']
    msgs = read_bag(args.bag, topics.values())
    imu = sorted((stamp(m), hull_attitude((m.orientation.x, m.orientation.y, m.orientation.z,
                                           m.orientation.w))) for _, m in msgs[topics['imu']])
    imu_t, imu_a = np.array([t for t, _ in imu]), [a for _, a in imu]
    print(f'hull attitude: FCU, level trim applied, {len(imu)} samples matched by stamp')
    frames = {m.header.frame_id for _, m in msgs[topics['points']][:1]}
    frames |= {'front_camera_link'} if msgs[topics['image']] else set()
    geometry, source = mounts(msgs[topics['tf_static']], frames)
    print(f'mounts from {source}')
    verdicts = []

    scans = []
    for _, m in msgs[topics['points']]:
        R, t = geometry.pose(m.header.frame_id)
        scans.append((stamp(m), read_points_numpy(m, field_names=('x', 'y', 'z'),
                                                  skip_nans=True) @ R.T + t))
    if args.buoy:
        x, y = (float(v) for v in args.buoy.split(','))
        good = [(dr, db) for dr, db, v in (taped_buoy(s, (x, y), cfg['buoy']) for _, s in scans)
                if v != UNOBSERVABLE]
        if good:
            dr, db = np.median(np.array(good), axis=0)
            ok = abs(dr) < cfg['buoy']['range_tolerance'] and abs(db) < cfg['buoy'][
                'bearing_tolerance']
            verdicts.append(OK if ok else ERROR)
            print(f'taped buoy: range {dr:+.3f} m bearing {angle(db)} over {len(good)} scans '
                  f'-> {verdicts[-1]}')
        else:
            verdicts.append(UNOBSERVABLE)
            print(f'taped buoy: no single buoy-shaped cluster within {cfg["buoy"]["gate"]} m '
                  f'-> {UNOBSERVABLE}')

    errors, heights = [], []
    for t, scan in scans:
        fit = water_tilt(scan, cfg['water'])
        hull = attitude_at(imu_t, imu_a, t, cfg['attitude_max_dt'])
        if fit and hull:
            errors.append((fit[0] - hull[0], fit[1] - hull[1]))
            heights.append(fit[2])
    if errors:
        error, spread, v = verdict(errors, cfg['verdict'])
        verdicts.append(v)
        print(f'lidar: mount error roll {angle(error[0])} pitch {angle(error[1])} '
              f'(scan sd {angle(spread)}), water z {np.median(heights):+.3f} m '
              f'over {len(errors)}/{len(scans)} scans -> {v}')
    else:
        verdicts.append(UNOBSERVABLE)
        print(f'lidar: no water plane all around the boat with an FCU attitude at its stamp in '
              f'{len(scans)} scans -> {UNOBSERVABLE}')

    infos = msgs[topics['camera_info']]
    errors = []
    urdf = (normal_roll_pitch(geometry.pose('front_camera_link')[0].T[:, 2])
            if 'front_camera_link' in frames else None)
    for _, m in msgs[topics['image']][::10] if infos else []:
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(m.encoding)
        hull = attitude_at(imu_t, imu_a, stamp(m), cfg['attitude_max_dt'])
        if channels is None or hull is None:
            continue
        image = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * channels]
        gray = image.reshape(m.height, m.width, channels)[:, :, :3].mean(axis=2)
        k = infos[-1][1].k
        estimate = horizon(gray, k[0], k[4], k[2], k[5], cfg['horizon'])
        if estimate:
            pitch, roll = estimate
            errors.append((roll - hull[0] - urdf[0], pitch - hull[1] - urdf[1]))
    if errors:
        error, spread, v = verdict(errors, cfg['verdict'])
        verdicts.append(v)
        print(f'camera: mount error roll {angle(error[0])} pitch {angle(error[1])} '
              f'(frame sd {angle(spread)}) over {len(errors)} frames -> {v}')
    else:
        verdicts.append(UNOBSERVABLE)
        print(f'camera: no unambiguous water edge with an FCU attitude at its stamp '
              f'-> {UNOBSERVABLE}')
    print('a mount error goes into blastoise.xacro lidar_rpy / camera_rpy (roll, pitch in rad)')
    if ERROR in verdicts:
        return 1
    return EXIT_UNOBSERVABLE if all(v == UNOBSERVABLE for v in verdicts) else 0


if __name__ == '__main__':
    sys.exit(main())
