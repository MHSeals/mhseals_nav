"""Publish the nearest lethal cell of the perception costmap, as seen from the hull.

The velocity controller's drift observer must not learn a dock's or a buoy's reaction as
current; it needs only where the closest obstacle is. One small Vector3Stamped in the base
frame, from the nearest point of the hull rectangle to the nearest lethal cell (NaN when
there is none within ``radius_m``). Nothing is published while the costmap or the TF is
missing or stale, so a subscriber sees silence, not a clear sky.
"""

import math
import time

import numpy


def nearest(cells, resolution, origin, pose, half, radius, lethal=100):
    """Body-frame vector from the hull to the nearest lethal cell, or None.

    ``cells`` is the grid (rows y, columns x) with ``origin`` its (x, y) corner in the
    grid frame, ``pose`` the base (x, y, yaw) in that frame and ``half`` the hull's
    (half length, half width). A cell under the hull reports 1 mm toward the cell.
    """
    x, y, yaw = pose
    reach = radius + math.hypot(*half)
    cols = slice(max(0, int((x - reach - origin[0]) / resolution)),
                 max(0, int(math.ceil((x + reach - origin[0]) / resolution))))
    rows = slice(max(0, int((y - reach - origin[1]) / resolution)),
                 max(0, int(math.ceil((y + reach - origin[1]) / resolution))))
    ys, xs = numpy.nonzero(cells[rows, cols] >= lethal)
    if not len(xs):
        return None
    dx = origin[0] + (xs + cols.start + 0.5) * resolution - x
    dy = origin[1] + (ys + rows.start + 0.5) * resolution - y
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    gx, gy = bx - numpy.clip(bx, -half[0], half[0]), by - numpy.clip(by, -half[1], half[1])
    gap = numpy.hypot(gx, gy)
    i = int(numpy.argmin(gap))
    if gap[i] > radius:
        return None
    if gap[i] > 0:
        return float(gx[i]), float(gy[i])
    size = math.hypot(bx[i], by[i])
    return (1e-3 * bx[i] / size, 1e-3 * by[i] / size) if size else (1e-3, 0.0)


class Grid:
    """The latest full costmap with its updates patched in."""

    def __init__(self):
        self.cells = None
        self.received = None

    def full(self, msg, now):
        info = msg.info
        self.frame = msg.header.frame_id
        self.resolution = info.resolution
        self.origin = (info.origin.position.x, info.origin.position.y)
        self.cells = numpy.asarray(msg.data, numpy.int8).reshape(info.height, info.width)
        self.received = now

    def update(self, msg, now):
        if self.cells is None or msg.y + msg.height > self.cells.shape[0] or \
                msg.x + msg.width > self.cells.shape[1]:
            return
        self.cells[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = numpy.asarray(
            msg.data, numpy.int8).reshape(msg.height, msg.width)
        self.received = now


def node_class():
    """Build the ROS node class lazily so the geometry imports without rclpy."""
    from geometry_msgs.msg import Vector3Stamped
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    from mhseals_nav.yaml_params import node_config

    class NearestObstacle(Node):
        def __init__(self):
            super().__init__('nearest_obstacle',
                             automatically_declare_parameters_from_overrides=True)
            self.cfg = cfg = node_config(self, 'nearest_obstacle.yaml')
            self.grid = Grid()
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.pub = self.create_publisher(Vector3Stamped, cfg['output_topic'], 10)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, cfg['costmap_topic'],
                                     lambda m: self.grid.full(m, time.monotonic()), latched)
            self.create_subscription(OccupancyGridUpdate, cfg['updates_topic'],
                                     lambda m: self.grid.update(m, time.monotonic()), 10)
            self.create_timer(1.0 / cfg['rate_hz'], self.tick)

        def tick(self):
            cfg, grid = self.cfg, self.grid
            if grid.cells is None or time.monotonic() - grid.received > cfg['max_age_s']:
                return
            try:
                tf = self.tf.lookup_transform(grid.frame, cfg['base_frame'], Time())
            except TransformException:
                return
            stamp = Time.from_msg(tf.header.stamp, clock_type=self.get_clock().clock_type)
            if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > cfg['max_age_s']:
                return
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            found = nearest(grid.cells, grid.resolution, grid.origin, (t.x, t.y, yaw),
                            (cfg['half_length_m'], cfg['half_width_m']), cfg['radius_m'],
                            cfg['lethal'])
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cfg['base_frame']
            msg.vector.x, msg.vector.y = found or (math.nan, math.nan)
            self.pub.publish(msg)

    return NearestObstacle


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = node_class()()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
