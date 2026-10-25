"""Hold the boat through command_mux outside a geofence, or near its edge heading outward.

The FCU fence is off and could not stop the ODROID-driven thrusters anyway. The fence is a
polygon in lat/lon (the map origin moves with every start, navsat wait_for_datum is false)
or in map metres; fence_file, the day's course area, overrides the package YAML. Position
comes from the EKF (/gps/filtered for lat/lon, /odom/global for map), heading and velocity
from /odom/global. No valid fence, or no fresh position or odometry, holds too.
"""

from array import array
import math
from pathlib import Path

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run
import yaml

EARTH_RADIUS_M = 6371000.0


def polygon(frame, points):
    """Vertices in a local metric plane (x east, y north); None unless a valid polygon."""
    if not isinstance(points, (list, tuple, array)):   # ROS double arrays come as array
        return None
    points = list(points)
    if frame not in ('latlon', 'map') or len(points) % 2 or len(points) < 6 or not all(
            type(v) in (int, float) and math.isfinite(v) for v in points):
        return None
    pairs = list(zip(points[::2], points[1::2]))
    if frame == 'map':
        return pairs
    origin = pairs[0]
    return [project(origin, lat, lon) for lat, lon in pairs]


def project(origin, lat, lon):
    """Equirectangular east/north metres from origin (lat, lon); exact enough for a course."""
    k = math.pi / 180.0 * EARTH_RADIUS_M
    return ((lon - origin[1]) * k * math.cos(math.radians(origin[0])), (lat - origin[0]) * k)


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
        self.frame = cfg['fence_frame']
        self.poly = polygon(self.frame, cfg.get('fence_points', []))
        self.p, self.p_at = None, None
        self.yaw, self.v, self.motion_at = 0.0, (0.0, 0.0), None

    def position(self, now, a, b):
        """(lat, lon) for a lat/lon fence, else (x, y) in map."""
        if self.poly is None:
            return
        points = self.cfg['fence_points']
        self.p = project(points[:2], a, b) if self.frame == 'latlon' else (a, b)
        self.p_at = now

    def motion(self, now, yaw, vx, vy):
        """Heading and body velocity; the velocity is rotated into the fence plane (ENU)."""
        c, s = math.cos(yaw), math.sin(yaw)
        self.yaw, self.v, self.motion_at = yaw, (vx * c - vy * s, vx * s + vy * c), now

    def reason(self, now):
        cfg = self.cfg
        if self.poly is None:
            return 'no valid fence configured'
        if not fresh(self.p_at, now, cfg['max_age_s']):
            return 'position missing or stale'
        if not fresh(self.motion_at, now, cfg['max_age_s']):
            return 'odometry missing or stale'
        if not inside(self.p, self.poly):
            return 'outside the fence'
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
                return f'{d:.1f} m from the fence heading outward'
        return ''


def load_fence(cfg):
    """Return cfg with fence_frame and fence_points from fence_file when that file exists.

    A file that cannot be read or does not parse to a mapping leaves no valid fence, which
    holds.
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
    return cfg


def node_class():
    import time
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    def logic(cfg):
        return FenceWatchdog(load_fence(cfg))

    def subscribe(node):
        cfg, dog = node.logic.cfg, node.logic
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
            if dog.frame == 'map':
                dog.position(now, msg.pose.pose.position.x, msg.pose.pose.position.y)

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
