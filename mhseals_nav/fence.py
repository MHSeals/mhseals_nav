"""The geofence itself: its geometry and the file that holds it.

Pure (no ROS), so fence_watchdog and the TUI's goal gates share one implementation.
"""

from array import array
import math
from pathlib import Path

import yaml

EARTH_RADIUS_M = 6371000.0
ON_EXIT = ('alert', 'return')


def problem(frame, points):
    """Why ``points`` is no valid fence in ``frame`` ('' when it is)."""
    if frame not in ('latlon', 'map'):
        return f'frame {frame!r} is neither latlon nor map'
    if not isinstance(points, (list, tuple, array)):   # ROS double arrays come as array
        return 'not a list'
    points = list(points)
    if len(points) % 2 or len(points) < 6:
        return 'a flat list of at least 3 corner pairs'
    if not all(type(v) in (int, float) and math.isfinite(v) for v in points):
        return 'not all finite numbers'
    pairs = list(zip(points[::2], points[1::2]))
    if frame == 'latlon':
        if not all(abs(lat) <= 90 and abs(lon) <= 180 for lat, lon in pairs):
            return 'a corner outside lat [-90, 90], lon [-180, 180]'
        pairs = [project(pairs[0], lat, lon) for lat, lon in pairs]
    return '' if simple(pairs) else 'not a simple polygon (repeated corner, no area or ' \
        'crossing edges)'


def polygon(frame, points):
    """Vertices in a local metric plane (x east, y north); None unless a valid polygon."""
    if problem(frame, points):
        return None
    pairs = [(float(a), float(b)) for a, b in zip(points[::2], points[1::2])]
    if frame == 'map':
        return pairs
    origin = pairs[0]
    return [project(origin, lat, lon) for lat, lon in pairs]


def simple(pairs):
    """Whether the closed polygon has distinct corners, an area and no edges that meet
    other than neighbours at their shared corner."""
    n = len(pairs)
    edges = [(pairs[k], pairs[(k + 1) % n]) for k in range(n)]
    if len(set(pairs)) < n:
        return False
    if abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)) < 1e-6:
        return False
    return not any(meet(*edges[i], *edges[j]) for i in range(n) for j in range(i + 2, n)
                   if not (i == 0 and j == n - 1))


def meet(a, b, c, d):
    """Whether segments ab and cd share a point."""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def within(p, q, r):
        return min(p[0], q[0]) <= r[0] <= max(p[0], q[0]) and \
            min(p[1], q[1]) <= r[1] <= max(p[1], q[1])
    s1, s2, s3, s4 = side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b)
    if s1 * s2 < 0 and s3 * s4 < 0:
        return True
    return any(s == 0 and within(p, q, r) for s, p, q, r in (
        (s1, a, b, c), (s2, a, b, d), (s3, c, d, a), (s4, c, d, b)))


def project(origin, lat, lon):
    """Equirectangular east/north metres from origin (lat, lon); exact enough for a course."""
    k = math.pi / 180.0 * EARTH_RADIUS_M
    return ((lon - origin[1]) * k * math.cos(math.radians(origin[0])), (lat - origin[0]) * k)


def reference(frame, points):
    """The point the configured fence is anchored by: the map origin (where the boat started,
    so the start of the test is the first anchoring) or, for an absolute lat/lon fence, the
    vertex mean."""
    if frame == 'map':
        return (0.0, 0.0)
    return (sum(points[::2]) / (len(points) // 2), sum(points[1::2]) / (len(points) // 2))


def moved(points, ref, at):
    """The flat vertex list translated so that ``ref`` lands on ``at`` (same frame)."""
    da, db = at[0] - ref[0], at[1] - ref[1]
    return [float(v) + (da if k % 2 == 0 else db) for k, v in enumerate(points)]


def outline(frame, points, boat_raw=None, boat_map=None):
    """Closed [(x, y), ...] of the fence in map, None when it cannot be placed there.

    A map fence is its own points; a lat/lon one is placed through the boat, seen at once in
    lat/lon (``boat_raw``) and in map (``boat_map``), east/north from the boat's fix.
    """
    if polygon(frame, points) is None:
        return None
    pairs = list(zip(points[::2], points[1::2]))
    if frame == 'latlon':
        if boat_raw is None or boat_map is None:
            return None
        pairs = [(boat_map[0] + e, boat_map[1] + n)
                 for e, n in (project(boat_raw, lat, lon) for lat, lon in pairs)]
    pairs = [(float(x), float(y)) for x, y in pairs]
    return pairs + pairs[:1]


def inside(p, poly):
    x, y = p
    result = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            result = not result
    return result


def closest(p, poly):
    """For each edge, the point of it closest to p."""
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = ((p[0] - x1) * dx + (p[1] - y1) * dy) / ((dx * dx + dy * dy) or 1.0)
        t = min(1.0, max(0.0, t))
        yield x1 + t * dx, y1 + t * dy


def load_fence(fence_file):
    """The fence ``fence_file`` holds: ({'frame', 'points', 'on_exit'}, '') or (None, why).

    ``{frame: latlon | map, points: [a, b, ...], on_exit: alert | return}`` (on_exit defaults
    to alert), or the TUI's ``course: {corners: [lat1, lon1, ...], on_exit}``, a lat/lon fence.
    ~ expands; a relative path is in this package's config/ (the simulator's fences/crane.yaml).
    No file, an unreadable or invalid one, is no fence, which holds.
    """
    path = resolve(fence_file)
    if path is None:
        return None, 'no fence_file'
    try:
        day = yaml.safe_load(path.read_text())
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        return None, f'{path}: {type(error).__name__}'
    if isinstance(day, dict) and isinstance(day.get('course'), dict):
        day = dict(day['course'], frame='latlon', points=day['course'].get('corners'))
    if not isinstance(day, dict):
        return None, f'{path}: not a mapping'
    on_exit = day.get('on_exit', 'alert')
    why = problem(day.get('frame'), day.get('points')) or (
        '' if on_exit in ON_EXIT else f'on_exit {on_exit!r} is neither alert nor return')
    if why:
        return None, f'{path}: {why}'
    return {'frame': day['frame'], 'points': [float(v) for v in day['points']],
            'on_exit': on_exit}, ''


def resolve(fence_file):
    """fence_file as a path (see ``load_fence``), None when empty."""
    if not fence_file:
        return None
    path = Path(fence_file).expanduser()
    if path.is_absolute():
        return path
    from mhseals_nav.yaml_params import config_path
    return config_path(path)


class FenceFile:
    """fence_file, read again whenever it changes (written or replaced): an edit applies live."""

    def __init__(self, fence_file):
        self.fence_file, self.seen = fence_file, ()      # () is no stat: the first poll reads

    def poll(self):
        """``load_fence``'s answer when the file changed since the last poll, else None."""
        try:
            info = resolve(self.fence_file).stat()
            seen = (info.st_ino, info.st_mtime_ns, info.st_size)
        except (AttributeError, OSError):       # no fence_file, or none there
            seen = None
        if seen == self.seen:
            return None
        self.seen = seen
        return load_fence(self.fence_file)
