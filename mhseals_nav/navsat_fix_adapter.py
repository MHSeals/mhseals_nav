#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)


class NavSatFixAdapter(Node):
    def __init__(self):
        super().__init__('navsat_fix_adapter')

        input_topic = '/mavros/global_position/global'
        output_topic = '/gps/fix'

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            NavSatFix,
            input_topic,
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            NavSatFix,
            output_topic,
            10
        )

        self.get_logger().info(
            f'NavSatFix adapter running:\n'
            f'  {input_topic}  ->  {output_topic}'
        )

    def callback(self, msg: NavSatFix):
        fixed = NavSatFix()

        # Preserve header & position
        fixed.header = msg.header
        fixed.latitude = msg.latitude
        fixed.longitude = msg.longitude
        fixed.altitude = msg.altitude

        # Force GPS to be valid
        fixed.status.status = NavSatStatus.STATUS_FIX
        fixed.status.service = NavSatStatus.SERVICE_GPS

        # Perfect simulation: zero covariance
        fixed.position_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]
        fixed.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        self.pub.publish(fixed)


def main():
    rclpy.init()
    node = NavSatFixAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()