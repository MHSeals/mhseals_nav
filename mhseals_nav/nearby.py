"""Summarize /points as the nearest range around the hull, on /perception/nearby.

The raw cloud (about 8 Mbit/s) cannot cross the radio (about 1 Mbit/s) to the TUI; a few
hundred bytes say what is close. std_msgs/String JSON, at most ``rate`` Hz and only for a new
cloud (silence is no data):

    {sectors: [nearest range per sector, from the bow clockwise, null when empty],
     nearest: {range, bearing} or null, count, rate, latency, frame}

Ranges are horizontal metres from base_link's origin; sector 0 is centred on the bow, each
360/n degrees wide; bearing is degrees clockwise from the bow in [0, 360). count is the points
kept after the height band and the hull, rate the clouds received per second, latency the
cloud's stamp age when summarized. The lidar mount comes from /tf_static, as in lidar_relay.
"""

import collections
import json
import time

import numpy

FLOATS = {7: 'f4', 8: 'f8'}   # PointField FLOAT32, FLOAT64


def datatype_size(field):
    return numpy.dtype(FLOATS[field.datatype]).itemsize


def xyz(cloud):
    """(N, 3) float64 finite x, y, z of a PointCloud2; empty without float x, y and z."""
    fields = {f.name: f for f in cloud.fields}
    if any(fields.get(n) is None or fields[n].datatype not in FLOATS for n in 'xyz'):
        return numpy.empty((0, 3))
    step, width, height = cloud.point_step, cloud.width, cloud.height
    if (step <= 0 or not width * height or cloud.row_step < width * step
            or len(cloud.data) < height * cloud.row_step
            or any(fields[n].offset < 0 or fields[n].offset + datatype_size(fields[n]) > step
                   for n in 'xyz')):
        return numpy.empty((0, 3))
    order = '>' if cloud.is_bigendian else '<'
    layout = numpy.dtype({'names': list('xyz'),
                          'formats': [order + FLOATS[fields[n].datatype] for n in 'xyz'],
                          'offsets': [fields[n].offset for n in 'xyz'], 'itemsize': step})
    rows = numpy.frombuffer(cloud.data, numpy.uint8, count=height * cloud.row_step)
    rows = numpy.ascontiguousarray(rows.reshape(height, cloud.row_step)[:, :width * step])
    points = rows.view(layout).reshape(-1)
    out = numpy.stack([points['x'], points['y'], points['z']], axis=1).astype(numpy.float64)
    return out[numpy.isfinite(out).all(axis=1)]


def summarize(body, sectors, low, high, hull):
    """Sector ranges, the nearest point and the count of base-frame points ``body``.

    Kept: z within [low, high] and outside the hull rectangle ``hull`` (half length, half
    width) about the origin.
    """
    x, y, z = body[:, 0], body[:, 1], body[:, 2]
    keep = (z >= low) & (z <= high) & ~((numpy.abs(x) <= hull[0]) & (numpy.abs(y) <= hull[1]))
    x, y = x[keep], y[keep]
    out = {'sectors': [None] * sectors, 'nearest': None, 'count': int(len(x))}
    if not len(x):
        return out
    ranges = numpy.hypot(x, y)
    bearing = numpy.degrees(numpy.arctan2(-y, x)) % 360.0
    width = 360.0 / sectors
    index = ((bearing + width / 2) // width).astype(int) % sectors
    best = numpy.full(sectors, numpy.inf)
    numpy.minimum.at(best, index, ranges)
    out['sectors'] = [round(float(r), 2) if numpy.isfinite(r) else None for r in best]
    i = int(numpy.argmin(ranges))
    out['nearest'] = {'range': round(float(ranges[i]), 2),
                      'bearing': round(float(bearing[i]), 1) % 360.0}
    return out


class Rate:
    """Arrivals per second over the last ``window`` seconds."""

    def __init__(self, window=2.0):
        self.window, self.times = window, collections.deque()

    def tick(self, now):
        self.times.append(now)
        while now - self.times[0] > self.window:
            self.times.popleft()

    def hz(self):
        span = self.times[-1] - self.times[0] if len(self.times) > 1 else 0.0
        return round((len(self.times) - 1) / span, 1) if span > 0 else 0.0


def node_class():
    """Build the ROS node class lazily so the summary imports without rclpy."""
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    from mhseals_nav.lidar_relay import add_static, lidar_mount
    from mhseals_nav.yaml_params import rectangle, node_section_config

    class Nearby(Node):
        def __init__(self):
            super().__init__('nearby', automatically_declare_parameters_from_overrides=True)
            # config/hull.yaml < config/lidar.yaml costmap: < launch overrides
            self.near = cfg = node_section_config(self, 'lidar.yaml', 'costmap', rectangle())
            self.hull = (cfg['half_length'] + cfg['hull_margin'],
                         cfg['half_width'] + cfg['hull_margin'])
            self.cloud, self.rate = None, Rate()
            self.static, self.mounts = {}, {}
            self.pub = self.create_publisher(String, self.near['topic'], 10)
            self.create_subscription(
                TFMessage, '/tf_static', lambda m: add_static(self.static, m),
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PointCloud2, self.near['cloud_topic'], self.received,
                                     qos_profile_sensor_data)
            self.create_timer(1.0 / self.near['rate'], self.tick)

        def received(self, cloud):
            self.cloud = cloud
            self.rate.tick(time.monotonic())

        def tick(self):
            cloud, self.cloud = self.cloud, None
            if cloud is None:
                return
            mount = lidar_mount(self.static, self.mounts, cloud.header.frame_id)
            if mount is None:
                self.get_logger().warn(f'no /tf_static mount for {cloud.header.frame_id}',
                                       throttle_duration_sec=10.0)
                return
            body = xyz(cloud) @ numpy.asarray(mount[0]).T + numpy.asarray(mount[1])
            out = summarize(body, int(self.near['sectors']), self.near['min_height'],
                            self.near['max_height'], self.hull)
            stamp = Time.from_msg(cloud.header.stamp, clock_type=self.get_clock().clock_type)
            out.update(rate=self.rate.hz(), frame=self.near['base_frame'], latency=round(
                (self.get_clock().now() - stamp).nanoseconds * 1e-9, 3))
            self.pub.publish(String(data=json.dumps(out)))

    return Nearby


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
