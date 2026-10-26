#!/usr/bin/env python3
"""Sim only: give crane_ml's instantaneous lidar scans the sweep of a real VLP16.

crane_ml casts every ray at the cloud's stamp. A VLP16 sweeps once per 0.1 s and the driver
stamps the scan at its last packet, each point carrying its offset from the stamp in a float32
``time`` field. This node gives each point the time the beam reaches its azimuth (sweep_times),
moves it into the lidar frame of that moment with the ground-truth pose (/crane/odom) and the
base_link -> lidar_link transform, and appends ``time``. A turning boat then smears /points as
on the water, and lidar_relay's deskew can be judged in sim. sensors.launch.py starts it when
config/lidar_smear.yaml enables it, with the endpoint's cloud moved to /points/instant.
"""

from collections import deque

from mhseals_nav.lidar_relay import _qos, SWEEP, sweep_times
import numpy as np

STEPS = 1000   # lidar poses per sweep


def planar(x, y, yaw, px, py):
    """(px, py) in a frame at (x, y, yaw) -> world."""
    c, s = np.cos(yaw), np.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def smear(x, y, poses, lever=(0., 0., 0.)):
    """Lidar points (x, y) seen all at once at t = 0 -> (x, y, t) as a sweep ending at 0 sees them.

    poses: (t, x, y, yaw) rows of base_link in the world, t relative to the stamp, sorted.
    lever: (x, y, yaw) of lidar_link in base_link.
    """
    times, bx, by, byaw = (np.asarray(c, dtype=np.float64) for c in zip(*poses))
    byaw = np.unwrap(byaw)

    def sensor(t):
        b = [np.interp(t, times, c) for c in (bx, by, byaw)]
        return (*planar(*b, lever[0], lever[1]), b[2] + lever[2])

    wx, wy = planar(*sensor(0.), x, y)
    # The lidar pose every 0.1 ms of the sweep; each point takes the nearest.
    grid = np.linspace(-SWEEP, 0., STEPS + 1)
    sx, sy, syaw = sensor(grid)
    c, s = np.cos(syaw), np.sin(syaw)
    t = sweep_times(x, y)
    for _ in range(2):  # the second pass times each point by the azimuth it is seen at
        i = np.rint((t / SWEEP + 1) * STEPS).astype(np.intp)
        dx, dy = wx - sx[i], wy - sy[i]
        px, py = c[i] * dx + s[i] * dy, c[i] * dy - s[i] * dx
        t = sweep_times(px, py)
    return px, py, grid[i]


def main():
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('lidar_smear')
    tf = Buffer()
    TransformListener(tf, node)
    odom = deque(maxlen=50)   # 1 s of /crane/odom at 50 Hz
    pending = []
    lever = []
    publisher = node.create_publisher(PointCloud2, '/points', _qos())
    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'time'))]

    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def flush():
        cloud = pending.pop()
        if not lever:
            try:
                q = tf.lookup_transform('base_link', cloud.header.frame_id, Time()).transform
            except Exception:  # noqa: B902 (no TF yet: smear without the lever arm)
                q = None
            if q is not None:
                lever.extend((q.translation.x, q.translation.y, 2 * np.arctan2(
                    q.rotation.z, q.rotation.w)))
        f = {f.name: f.offset for f in cloud.fields}
        n = cloud.width * cloud.height
        p = np.frombuffer(cloud.data, dtype={'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                                             'offsets': [f['x'], f['y'], f['z']],
                                             'itemsize': cloud.point_step}, count=n)
        out = np.empty((n, 4), dtype='<f4')
        stamp = seconds(cloud.header.stamp)
        poses = [(t - stamp, *pose) for t, *pose in odom]
        if len(poses) < 2:
            return
        out[:, 0], out[:, 1], out[:, 3] = smear(
            p['x'].astype(np.float64), p['y'].astype(np.float64), poses, lever or (0, 0, 0))
        out[:, 2] = p['z']
        publisher.publish(PointCloud2(
            header=cloud.header, height=1, width=n, fields=fields, is_bigendian=False,
            point_step=16, row_step=16 * n, data=out.tobytes(), is_dense=cloud.is_dense))

    def on_cloud(cloud):
        if pending:      # its pose never came: smear it with what there is
            flush()
        pending.append(cloud)

    def on_odom(m):
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        odom.append((seconds(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        if pending and odom[-1][0] >= seconds(pending[0].header.stamp):
            flush()

    node.create_subscription(PointCloud2, '/points/instant', on_cloud, _qos(False))
    node.create_subscription(Odometry, '/crane/odom', on_odom, _qos(False))
    rclpy.spin(node)


if __name__ == '__main__':
    main()
