#!/usr/bin/env python3
"""Relay the large-data lidar cloud into the default DDS graph for RViz and Nav2.

Fast DDS picks its transport once per process, so the relay runs two: a receiver
on config/fastdds_large_data.xml and a publisher on the default transport. The
receiver passes each cloud, decimated to a bounded payload, through a pipe.
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
    data = b"".join(bytes(message.data[i * step:(i + 1) * step]) for i in range(0, total, stride))
    return type(message)(
        header=message.header, height=1, width=len(data) // step, fields=message.fields,
        is_bigendian=message.is_bigendian, point_step=step, row_step=len(data), data=data,
        is_dense=message.is_dense)


def _qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    # Reliable is compatible with both RViz's reliable reader and Nav2's
    # best-effort SensorDataQoS reader, so one writer serves both.
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
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
        lambda message: pipe.send_bytes(serialize_message(decimate(message, max_bytes))), _qos())
    node.get_logger().info(f"receiving {topic} on the large-data transport")
    rclpy.spin(node)


def publish(pipe, topic):
    os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("lidar_relay_publish")
    publisher = node.create_publisher(PointCloud2, topic, _qos())
    node.get_logger().info(f"publishing {topic} on the default transport")
    while rclpy.ok():
        if not pipe.poll(0.5):
            continue
        # Publish only the newest cloud so a slow reader never sees stale scans.
        payload = pipe.recv_bytes()
        while pipe.poll(0):
            payload = pipe.recv_bytes()
        publisher.publish(deserialize_message(payload, PointCloud2))


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
