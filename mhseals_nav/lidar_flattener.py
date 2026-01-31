#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class LidarFlattener(Node):
    def __init__(self):
        super().__init__('lidar_flattener')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        self.declare_parameter('hz', 10)
        self.declare_parameter('angle_increment', 1)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 100.0)

        self.hz = self.get_parameter('hz').get_parameter_value().integer_value
        self.angle_increment = np.deg2rad(
            self.get_parameter('angle_increment').get_parameter_value().integer_value
        )
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        self.sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.point_cloud_callback,
            qos
        )
        self.get_logger().info("Subscribed to /points topic!")

        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos
        )
        self.get_logger().info("Publishing received data to /scan...")

    # Assumes standard REP-103 convention (x, y, z) -> (forward, left, up)
    def point_cloud_callback(self, pc: PointCloud2):
        pts = np.array(
            list(point_cloud2.read_points(
                pc,
                field_names=['x', 'y', 'z'],
                skip_nans=True
            ))
        )
        
        if pts.size == 0:
            return
        
        x = pts['x'].astype(np.float32)
        y = pts['y'].astype(np.float32)
    
        angles = np.arctan2(y, x)
        ranges = np.sqrt(x**2 + y**2)
    
        angle_min = -np.pi
        angle_max = np.pi
    
        num_beams = int((angle_max - angle_min) / self.angle_increment)
    
        scan_ranges = np.full(num_beams, np.inf, dtype=np.float32)

        indices = ((angles - angle_min) / self.angle_increment).astype(np.int32)
    
        valid = (
            (indices >= 0) &
            (indices < num_beams) &
            (ranges >= self.range_min) &
            (ranges <= self.range_max)
        )
    
        for i, r in zip(indices[valid], ranges[valid]):
            if r < scan_ranges[i]:
                scan_ranges[i] = r
    
        scan = LaserScan()
        scan.header = pc.header
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = scan_ranges.tolist()
    
        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFlattener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()