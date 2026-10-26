#!/usr/bin/env python3
"""Relay the large-data lidar cloud into the default DDS graph for RViz and Nav2.

Fast DDS picks its transport once per process, so the relay runs two: a receiver
on config/fastdds_large_data.xml and a publisher on the default transport. The
receiver passes each cloud, decimated to a bounded payload, through a pipe.
Run one relay on the sensor host; its normal-DDS output serves Nav2 and viewers.
"""

import argparse
import multiprocessing
import multiprocessing.connection
import os
import signal
import sys

from ament_index_python.packages import get_package_share_directory


def decimate(message, max_bytes):
    # A VLP16 cloud is ~642 KiB, which the default UDP transport cannot carry
    # reliably. Keep whole points at an even stride so the fields, frame and
    # PointCloud2 contract stay unchanged for costmaps and RViz.
    if not max_bytes or len(message.data) <= max_bytes or message.point_step <= 0:
        return message
    step = message.point_step
    total = message.width * message.height
    stride = max(1, -(-total // max(1, max_bytes // step)))
    # VLP16 points interleave 16 laser rings. An even stride (typically 20)
    # aliases that pattern and loses entire rings. An odd stride visits all
    # 16 rings while keeping the same payload bound.
    if stride % 2 == 0:
        stride += 1
    data = b"".join(bytes(message.data[i * step:(i + 1) * step]) for i in range(0, total, stride))
    return type(message)(
        header=message.header, height=1, width=len(data) // step, fields=message.fields,
        is_bigendian=message.is_bigendian, point_step=step, row_step=len(data), data=data,
        is_dense=message.is_dense)


TWIST_MAX_AGE = 0.3   # s; an older twist leaves the cloud as scanned
SWEEP = 0.1           # s per VLP16 revolution at 600 rpm


def sweep_times(x, y, period=SWEEP):
    """Time of each point from its azimuth, as an offset from a stamp at the end of the sweep.

    A VLP16 turns clockwise seen from above. With the seam behind the sensor, a sweep starts
    at azimuth +pi (t = -period), passes port, the bow and starboard, and ends at -pi (t = 0).
    The driver's seam drifts with packet count, so this only stands in for its ``time`` field.
    """
    import numpy as np
    return -period * (np.arctan2(y, x) + np.pi) / (2 * np.pi)


def fresh_twist(twist, now):
    """Return the latest (yaw_rate, velocity) if under TWIST_MAX_AGE old at now, else None."""
    if 'at' in twist and now - twist['at'] < TWIST_MAX_AGE:
        return twist['w'], twist['v']
    return None


def deskew(message, yaw_rate, velocity):
    """Move each point to where the sensor saw it at the header stamp.

    A VLP16 scan takes 0.1 s and the driver stamps it at its last packet, with each point's
    offset from the stamp in the float32 ``time`` field (-0.1..0 s). Turning at w, a point seen
    t earlier is off by w*t in azimuth: 0.3 m at 10 m range at autonomy's 0.3 rad/s, 1.5 m at
    the 1.5 rad/s of a manual spin. Constant body twist over the scan (yaw rate and planar
    velocity, base_link axes, which lidar_link shares): p_stamp = R(w t) p + v t. The lidar's
    lever arm adds w*0.29 m/s of sway, under 3 cm per scan at 1 rad/s, and is left out.
    Clouds without x, y and time fields pass unchanged.
    """
    import numpy as np
    fields = {f.name: f for f in message.fields}
    if not {'x', 'y', 'time'} <= set(fields) or any(
            fields[name].datatype != 7 for name in ('x', 'y', 'time')):  # 7: FLOAT32
        return message
    count = message.width * message.height
    order = '>' if message.is_bigendian else '<'
    layout = np.dtype({'names': ['x', 'y', 'time'], 'formats': [order + 'f4'] * 3,
                       'offsets': [fields[n].offset for n in ('x', 'y', 'time')],
                       'itemsize': message.point_step})
    data = bytearray(message.data)
    # A view over our own copy: assigning through it rewrites the payload.
    points = np.frombuffer(data, dtype=layout, count=count)
    t = points['time'].astype(np.float64)
    x, y = points['x'].astype(np.float64), points['y'].astype(np.float64)
    c, s = np.cos(yaw_rate * t), np.sin(yaw_rate * t)
    points['x'] = c * x - s * y + velocity[0] * t
    points['y'] = s * x + c * y + velocity[1] * t
    return type(message)(
        header=message.header, height=message.height, width=message.width,
        fields=message.fields, is_bigendian=message.is_bigendian,
        point_step=message.point_step, row_step=message.row_step, data=bytes(data),
        is_dense=message.is_dense)


def _qos(reliable=True):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    # Reliable is compatible with both RViz's reliable reader and Nav2's
    # best-effort SensorDataQoS reader, so one writer serves both.
    return QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE if reliable
                                           else ReliabilityPolicy.BEST_EFFORT),
                      durability=DurabilityPolicy.VOLATILE)


def receive(pipe, topic, max_bytes, profile):
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = profile
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("lidar_relay_receive")
    node.create_subscription(
        PointCloud2, topic,
        lambda message: pipe.send_bytes(serialize_message(decimate(message, max_bytes))), _qos(False))
    node.get_logger().info(f"receiving {topic} on the large-data transport")
    rclpy.spin(node)


def publish(pipe, topic):
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    # Reduce on the boat before crossing the LAN. Viewers subscribe directly;
    # another TCP reader of the full cloud can stall the sensor participant.
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
    os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "ASYNCHRONOUS"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    import time
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("lidar_relay_publish")
    publisher = node.create_publisher(PointCloud2, topic, _qos())
    # The fused body twist (ekf_local, 100 Hz) for deskew; older than TWIST_MAX_AGE, none.
    twist = {}
    node.create_subscription(Odometry, '/odom/local', lambda m: twist.update(
        w=m.twist.twist.angular.z, v=(m.twist.twist.linear.x, m.twist.twist.linear.y),
        at=time.monotonic()), _qos(False))
    node.get_logger().info(f'publishing {topic} on the default transport, deskewed')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        if not pipe.poll(0.01):
            continue
        # Publish only the newest cloud so a slow reader never sees stale scans.
        payload = pipe.recv_bytes()
        while pipe.poll(0):
            payload = pipe.recv_bytes()
        cloud = deserialize_message(payload, PointCloud2)
        motion = fresh_twist(twist, time.monotonic())
        if motion:
            cloud = deskew(cloud, *motion)
        publisher.publish(cloud)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/points")
    parser.add_argument("--max-bytes", type=int, default=32 * 1024,
                        help="cloud payload cap for the default transport")
    args, _ = parser.parse_known_args()
    profile = os.path.join(get_package_share_directory("mhseals_nav"), "config", "fastdds_large_data.xml")

    # Spawned children start without this process's DDS state, so each one
    # applies its own transport before rclpy creates a participant.
    context = multiprocessing.get_context("spawn")
    reader, writer = context.Pipe(duplex=False)
    children = [
        context.Process(target=receive, args=(writer, args.topic, args.max_bytes, profile), daemon=True),
        context.Process(target=publish, args=(reader, args.topic), daemon=True),
    ]
    for child in children:
        child.start()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        # If either half dies, exit so the container restart policy restarts both.
        multiprocessing.connection.wait([child.sentinel for child in children])
    except KeyboardInterrupt:
        pass
    finally:
        for child in children:
            child.terminate()
    sys.exit(1)


if __name__ == "__main__":
    main()
