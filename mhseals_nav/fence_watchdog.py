"""Hold the boat through command_mux outside a geofence, or near its edge heading outward.

The FCU fence is off and could not stop the ODROID-driven thrusters anyway. The fence is a
polygon in lat/lon (the map origin moves with every start, navsat wait_for_datum is false)
or in map metres; fence_file, the day's course area, overrides the package YAML. Position
comes from the EKF (/gps/filtered for lat/lon, /odom/global for map), heading and velocity
from /odom/global. No valid fence, or no fresh position or odometry, holds too.

It also publishes the fence it enforces on fence_topic (latched JSON, see ``state``): the raw
polygon in its own frame and the margin rules, so the TUI's goal gates check goals against this
one source (a lat/lon fence in lat/lon, through navsat's toLL, never a translated copy).

anchor_service (std_srvs/Trigger) re-anchors a map fence at the stopped boat (see ``anchor``);
a lat/lon course area stays where it is. marker_topic draws the fence for rviz, latched, in
marker_frame, marker_outside_color while the boat is outside.

The course area is live: the ``course.corners`` parameter ([lat1, lon1, lat2, lon2, ...], at
least 3 corners, a simple polygon) replaces the fence atomically or is refused with the reason,
and reads back what is enforced. ``on_exit`` (alert | return) is live too: alert holds outside;
return, under autonomy, sends Nav2 one slow goal (return_speed) to the nearest interior point
(return_inset in), and holds as alert does once it is not back inside within return_timeout,
the operator (or any hold) takes the mux, or the goal ends elsewhere. status_topic carries
``status`` at status_rate and on every change.
"""

from array import array
import math
from pathlib import Path

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run
import yaml

EARTH_RADIUS_M = 6371000.0
NEAR_EDGE = 'within the margin heading outward'
OUTSIDE = 'outside the fence'
INSIDE = ('', NEAR_EDGE, 'on the fence')      # reasons with the boat in the fence
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


class FenceWatchdog(HoldGate):
    def __init__(self, cfg):
        super().__init__(cfg)
        if cfg['on_exit'] not in ON_EXIT:
            raise ValueError(f"on_exit: {cfg['on_exit']!r} is neither alert nor return")
        self.frame = cfg['fence_frame']
        self.poly = polygon(self.frame, cfg.get('fence_points', []))
        self.ref = reference(self.frame, cfg['fence_points']) if self.poly else None
        self.p, self.p_at, self.raw = None, None, None
        self.fix_ll, self.fix_at = None, None
        self.map_p, self.map_at = None, None
        self.yaw, self.v, self.motion_at = 0.0, (0.0, 0.0), None
        self.edge_m = None
        self.returning = None    # {'at', 'goal'} while a return goal is out
        self.return_failed = False   # until back inside: hold as alert does
        self.orders = []         # for the node: (go, (x, y, yaw)), (cancel, why), (failed, why)

    def position(self, now, a, b):
        """(lat, lon) for a lat/lon fence, else (x, y) in map."""
        if self.frame == 'latlon':
            self.fix_ll, self.fix_at = (a, b), now
        if self.poly is None:
            return
        points = self.cfg['fence_points']
        self.p = project(points[:2], a, b) if self.frame == 'latlon' else (a, b)
        self.p_at, self.raw = now, (a, b)

    def fix(self, now, lat, lon):
        """The boat's lat/lon, kept whatever the frame, so a course area set live has it."""
        self.fix_ll, self.fix_at = (lat, lon), now
        if self.frame == 'latlon':
            self.position(now, lat, lon)

    def map_position(self, now, x, y):
        """The boat in map, which places a lat/lon fence's marker."""
        self.map_p, self.map_at = (x, y), now
        if self.frame == 'map':
            self.position(now, x, y)

    def set_corners(self, corners):
        """Make the lat/lon ``corners`` the fence, whole or not at all: the reason, '' when set."""
        why = problem('latlon', corners)
        if why:
            return why
        points = [float(v) for v in corners]
        self.cfg = dict(self.cfg, fence_frame='latlon', fence_points=points)
        self.frame, self.poly = 'latlon', polygon('latlon', points)
        self.ref = reference('latlon', points)
        self.p, self.p_at, self.raw = None, None, None
        if self.fix_ll is not None:          # the plane's origin is the first corner
            self.position(self.fix_at, *self.fix_ll)
        if self.returning is not None:       # aimed at the old fence: plan again
            self.returning = None
            self.orders.append(('cancel', 'the course area changed'))
        return ''

    def set_on_exit(self, mode):
        """alert or return: the reason, '' when set."""
        if mode not in ON_EXIT:
            return f'{mode!r} is neither alert nor return'
        self.cfg = dict(self.cfg, on_exit=mode)
        return ''

    def anchor(self, now):
        """Re-anchor the fence at the boat: (True, what) or (False, why not).

        A map fence's configured shape moves so its reference (``reference``) sits at the
        boat, as the start of the test placed it; margins, frame and shape stay as configured.
        A lat/lon course area is geographic: it stays where it is.
        """
        if self.poly is None:
            return False, 'no valid fence configured'
        if self.frame == 'latlon':
            return True, 'course area fixed'
        if not fresh(self.p_at, now, self.cfg['max_age_s']):
            return False, 'no boat position (missing or older than max_age_s)'
        if not fresh(self.motion_at, now, self.cfg['max_age_s']):
            return False, 'no odometry (missing or older than max_age_s)'
        if math.hypot(*self.v) > self.cfg['anchor_max_speed_mps']:
            return False, f'boat moving ({math.hypot(*self.v):.1f} m/s)'
        points = moved(self.cfg['fence_points'], self.ref, self.raw)
        self.cfg = dict(self.cfg, fence_points=points)
        self.poly, self.ref = polygon(self.frame, points), self.raw
        self.position(self.p_at, *self.raw)       # the plane's origin moved with the fence
        return True, f'fence anchored at {self.raw[0]:.7g}, {self.raw[1]:.7g} ({self.frame})'

    def outline(self, now):
        """The marker's closed outline in map (see ``outline``), None while not placeable."""
        limit = self.cfg['max_age_s']
        live = fresh(self.p_at, now, limit) and fresh(self.map_at, now, limit)
        return outline(self.frame, self.cfg['fence_points'],
                       self.raw if live else None, self.map_p if live else None)

    def motion(self, now, yaw, vx, vy):
        """Heading and body velocity; the velocity is rotated into the fence plane (ENU)."""
        c, s = math.cos(yaw), math.sin(yaw)
        self.yaw, self.v, self.motion_at = yaw, (vx * c - vy * s, vx * s + vy * c), now

    def state(self, now, stamp_ns=0):
        """Return what fence_topic carries: the raw fence it enforces and its margin rules.

        Always the same keys. ``frame`` is latlon ([lat, lon, ...]; a reader projects with
        ``project`` from the first vertex, as here) or map ([x, y, ...]); ``points`` is the
        configured flat list when ``valid``, else None with ``error`` saying why. ``reason`` is
        the hold reason now ('' while all is well); ``stamp_ns`` the node's clock. A reader
        treats a message older than three periods (3 / fence_rate_hz) as no fence.
        """
        cfg, valid = self.cfg, self.poly is not None
        return {'valid': valid, 'frame': self.frame,
                'points': [float(v) for v in cfg['fence_points']] if valid else None,
                'margin_m': cfg['margin_m'], 'outward_angle_deg': cfg['outward_angle_deg'],
                'outward_speed_mps': cfg['outward_speed_mps'], 'reason': self.reason(now),
                'error': '' if valid else 'no valid fence configured', 'stamp_ns': stamp_ns}

    def reason(self, now):
        cfg = self.cfg
        if self.poly is None:
            return 'no valid fence configured'
        if not fresh(self.p_at, now, cfg['max_age_s']):
            return 'position missing or stale'
        if not fresh(self.motion_at, now, cfg['max_age_s']):
            return 'odometry missing or stale'
        if not inside(self.p, self.poly):
            return OUTSIDE
        # Every edge within the margin counts, so a corner is covered from both sides.
        cos_limit = math.cos(math.radians(cfg['outward_angle_deg']))
        for c in closest(self.p, self.poly):
            dx, dy = c[0] - self.p[0], c[1] - self.p[1]
            d = math.hypot(dx, dy)
            if d >= cfg['margin_m']:
                continue
            if d < 1e-6:
                return 'on the fence'
            nx, ny = dx / d, dy / d
            if self.v[0] * nx + self.v[1] * ny > cfg['outward_speed_mps'] or \
                    math.cos(self.yaw) * nx + math.sin(self.yaw) * ny > cos_limit:
                self.edge_m = d    # kept out of the reason, which is compared every tick
                return NEAR_EDGE
        return ''

    def describe(self, why):
        return f'{why} ({self.edge_m:.1f} m)' if why == NEAR_EDGE else why

    def status(self, now):
        """What status_topic carries. ``inside`` and ``distance_to_edge`` (m, to the nearest
        edge, either side) are None without a fence or a fresh position; ``corners`` is the
        flat list in ``frame`` (latlon: lat, lon; map: x, y), None without a valid fence."""
        valid = self.poly is not None
        live = valid and fresh(self.p_at, now, self.cfg['max_age_s'])
        edge = min(math.dist(c, self.p) for c in closest(self.p, self.poly)) if live else None
        return {'inside': inside(self.p, self.poly) if live else None,
                'distance_to_edge': round(edge, 2) if live else None, 'frame': self.frame,
                'on_exit': self.cfg['on_exit'],
                'corners': [float(v) for v in self.cfg['fence_points']] if valid else None,
                'returning': self.returning is not None}

    def due(self, now):
        """A hold, except outside while a return goal is out (on_exit: return)."""
        self.step(now)
        if self.returning is not None and self.reason(now) == OUTSIDE:
            return False
        return super().due(now)

    def step(self, now):
        """Advance the return (on_exit: return), leaving the node its orders."""
        why = self.reason(now)
        if why in INSIDE:
            self.return_failed = False
        if self.returning is None:
            if why == OUTSIDE and self.cfg['on_exit'] == 'return' and \
                    not self.return_failed and self.autonomy(now):
                goal = self.return_goal(now)
                if goal is None:
                    self.fail('no interior point to return to (or no map pose)')
                else:
                    self.returning = {'at': now, 'goal': goal}
                    self.orders.append(('go', goal))
            return
        if self.cfg['on_exit'] != 'return':
            self.stop('on_exit is alert', failed=False)
        elif why not in INSIDE and why != OUTSIDE:
            self.stop(why, failed=True)
        elif self.outranked(now):
            self.stop(f'the mux input is {self.mux_input}', failed=True)
        elif why == OUTSIDE and now - self.returning['at'] > self.cfg['return_timeout']:
            self.stop('not back inside within return_timeout', failed=True)

    def stop(self, why, failed):
        self.returning = None
        self.orders.append(('cancel', why))
        if failed:
            self.fail(why)

    def fail(self, why):
        self.return_failed = True
        self.orders.append(('failed', why))

    def return_result(self, now, outcome):
        """The return goal ended (``outcome``: succeeded, aborted, rejected, ...); still
        outside then holds."""
        if self.returning is None:
            return
        self.returning = None
        self.orders.append(('cancel', f'goal {outcome}'))   # nothing to cancel; lifts the cap
        if self.reason(now) == OUTSIDE:
            self.fail(f'the return goal ended outside ({outcome})')

    def return_goal(self, now):
        """(x, y, yaw) in marker_frame: return_inset inside the nearest edge point, facing
        it from the boat; None when there is no such point or no fresh map pose."""
        if self.frame == 'latlon' and not fresh(self.map_at, now, self.cfg['max_age_s']):
            return None
        p, poly, inset = self.p, self.poly, self.cfg['return_inset']
        c = min(closest(p, poly), key=lambda q: math.dist(q, p))
        mean = (sum(x for x, _ in poly) / len(poly), sum(y for _, y in poly) / len(poly))
        target = None
        for toward in ((c[0] - p[0], c[1] - p[1]), (mean[0] - c[0], mean[1] - c[1])):
            norm = math.hypot(*toward)
            if norm > 1e-6:
                t = (c[0] + inset * toward[0] / norm, c[1] + inset * toward[1] / norm)
                if inside(t, poly):
                    target = t
                    break
        if target is None:
            return None
        yaw = math.atan2(target[1] - p[1], target[0] - p[0])
        if self.frame == 'latlon':      # map is east/north through the boat, as ``outline``
            target = (self.map_p[0] + target[0] - p[0], self.map_p[1] + target[1] - p[1])
        return target[0], target[1], yaw


def load_fence(cfg):
    """Return cfg with fence_frame and fence_points from fence_file when that file exists.

    A file that cannot be read or does not parse to a mapping leaves no valid fence, which
    holds. ``course: {corners: [lat1, lon1, ...]}`` (the TUI's course area, in the file or a
    ``course.corners`` parameter) is a lat/lon fence and wins over fence_frame/fence_points.
    """
    path = Path(cfg['fence_file']).expanduser() if cfg['fence_file'] else None
    if path is not None and path.is_file():
        try:
            day = yaml.safe_load(path.read_text())
        except (OSError, UnicodeDecodeError, yaml.YAMLError):
            day = None
        if not isinstance(day, dict):
            return dict(cfg, fence_points=None)
        cfg = dict(cfg, fence_frame=day.get('fence_frame', cfg['fence_frame']),
                   fence_points=day.get('fence_points'))
        if isinstance(day.get('course'), dict) and 'corners' in day['course']:
            cfg = dict(cfg, fence_frame='latlon', fence_points=day['course']['corners'])
    course = cfg.get('course')
    if isinstance(course, dict) and course.get('corners') is not None:
        cfg = dict(cfg, fence_frame='latlon', fence_points=list(course['corners']))
    return cfg


def node_class():
    import json
    import time
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import ColorRGBA, String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker

    def logic(cfg):
        return FenceWatchdog(load_fence(cfg))

    def subscribe(node):
        cfg, dog = node.logic.cfg, node.logic
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        fence = node.create_publisher(String, cfg['fence_topic'], latched)
        marker = node.create_publisher(Marker, cfg['marker_topic'], latched)
        drawn = []

        def publish_marker(now):
            line = dog.outline(now)
            if line is None or (drawn and drawn[-1] == line):
                return
            msg = Marker(ns='fence', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
            msg.header.frame_id = cfg['marker_frame']      # stamp 0: rviz's latest TF
            msg.pose.orientation.w = 1.0
            msg.scale.x = float(cfg['marker_width'])
            msg.color = ColorRGBA(**dict(zip('rgba', map(float, cfg['marker_color']))))
            msg.points = [Point(x=x, y=y) for x, y in line]
            marker.publish(msg)
            drawn[:] = [line]

        def publish_fence():     # its own timer: a failure here never touches the holds
            try:
                now = time.monotonic()
                state = dog.state(now, node.get_clock().now().nanoseconds)
                fence.publish(String(data=json.dumps(state)))
                publish_marker(now)
            except Exception as error:      # noqa: BLE001 - logged, the hold timer goes on
                node.get_logger().error(f'fence publish failed: {error}',
                                        throttle_duration_sec=10.0)
        node.create_timer(1.0 / cfg['fence_rate_hz'], publish_fence)

        def on_anchor(_, response):
            response.success, response.message = dog.anchor(time.monotonic())
            if response.success:
                node.get_logger().info(response.message)
                drawn.clear()
                publish_fence()
            else:
                node.get_logger().warning(f'fence anchor refused: {response.message}')
            return response
        node.create_service(Trigger, cfg['anchor_service'], on_anchor)
        if dog.poly is None:
            node.get_logger().error(f"no valid fence (fence_file {cfg['fence_file']!r}): "
                                    'autonomy will be held')
        else:
            node.get_logger().info(f'fence: {len(dog.poly)} vertices in {dog.frame}')

        def on_odom(msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            now = time.monotonic()
            dog.motion(now, yaw, msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            dog.map_position(now, x, y)
            if dog.frame == 'map':
                dog.position(now, x, y)

        def on_fix(msg):
            if msg.status.status >= NavSatStatus.STATUS_FIX and \
                    math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                dog.position(time.monotonic(), msg.latitude, msg.longitude)

        node.create_subscription(Odometry, cfg['odom_topic'], on_odom, qos_profile_sensor_data)
        if dog.frame == 'latlon':
            node.create_subscription(NavSatFix, cfg['gps_topic'], on_fix,
                                     qos_profile_sensor_data)

    return watchdog_class('fence_watchdog', 'fence_watchdog.yaml', logic, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
