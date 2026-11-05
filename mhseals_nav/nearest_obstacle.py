"""Publish the nearest lethal cell of the perception costmap, as seen from the hull.

The velocity controller's drift observer must not learn a dock's or a buoy's reaction as
current; it needs only where the closest obstacle is. One small Vector3Stamped in the base
frame, from the nearest point of the hull rectangle to the nearest lethal cell (NaN when
there is none within ``radius``). Nothing is published while the costmap or the TF is
missing or stale, so a subscriber sees silence, not a clear sky.
"""

import math
import time

from mhseals_nav.robot_geometry import yaw
import numpy


def lethal_points(cells, resolution, origin, centre, radius, lethal=100):
    """World (x, y) of the lethal cells within ``radius`` of ``centre``, as an (N, 2) array.

    ``cells`` is the grid (rows y, columns x) with ``origin`` its (x, y) corner.
    """
    x, y = centre
    cols = slice(max(0, int((x - radius - origin[0]) / resolution)),
                 max(0, int(math.ceil((x + radius - origin[0]) / resolution))))
    rows = slice(max(0, int((y - radius - origin[1]) / resolution)),
                 max(0, int(math.ceil((y + radius - origin[1]) / resolution))))
    ys, xs = numpy.nonzero(cells[rows, cols] >= lethal)
    return numpy.column_stack((origin[0] + (xs + cols.start + 0.5) * resolution,
                               origin[1] + (ys + rows.start + 0.5) * resolution))


def body(points, pose):
    """World points -> body-frame (bx, by) at ``pose`` (x, y, yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    return c * dx + s * dy, -s * dx + c * dy


def hull_gap(points, pose, half):
    """Distance from the hull rectangle (half length, width) at ``pose`` to each point."""
    bx, by = body(points, pose)
    return numpy.hypot(bx - numpy.clip(bx, -half[0], half[0]),
                       by - numpy.clip(by, -half[1], half[1]))


def nearest(cells, resolution, origin, pose, half, radius, lethal=100):
    """Body-frame vector from the hull to the nearest lethal cell, or None.

    ``pose`` is the base (x, y, yaw) in the grid frame and ``half`` the hull's
    (half length, half width). A cell under the hull reports 1 mm toward the cell.
    """
    points = lethal_points(cells, resolution, origin, pose[:2], radius + math.hypot(*half),
                           lethal)
    if not len(points):
        return None
    gap = hull_gap(points, pose, half)
    i = int(numpy.argmin(gap))
    if gap[i] > radius:
        return None
    bx, by = (float(v[0]) for v in body(points[i:i + 1], pose))
    if gap[i] > 0:
        return bx - min(max(bx, -half[0]), half[0]), by - min(max(by, -half[1]), half[1])
    size = math.hypot(bx, by)
    return (1e-3 * bx / size, 1e-3 * by / size) if size else (1e-3, 0.0)


def grid_pose(node, tf, grid, base_frame, max_grid_age, max_tf_age):
    """Return the base (x, y, yaw) in the grid frame, None while the grid or TF is stale."""
    from rclpy.time import Time
    from tf2_ros import TransformException

    if grid.cells is None or time.monotonic() - grid.received > max_grid_age:
        return None
    try:
        found = tf.lookup_transform(grid.frame, base_frame, Time())
    except TransformException:
        return None
    clock = node.get_clock()
    stamp = Time.from_msg(found.header.stamp, clock_type=clock.clock_type)
    if (clock.now() - stamp).nanoseconds * 1e-9 > max_tf_age:
        return None
    t = found.transform.translation
    return t.x, t.y, yaw(found.transform.rotation)


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
    from tf2_ros import Buffer, TransformListener

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
            self.create_timer(1.0 / cfg['rate'], self.tick)

        def tick(self):
            cfg, grid = self.cfg, self.grid
            pose = grid_pose(self, self.tf, grid, cfg['base_frame'], cfg['max_age'],
                             cfg['max_age'])
            if pose is None:
                return
            found = nearest(grid.cells, grid.resolution, grid.origin, pose,
                            (cfg['half_length'], cfg['half_width']), cfg['radius'],
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
