#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose

class ZedDetectionsConverter(Node):

    def __init__(self):
        super().__init__('zed_detections_converter')

        self.sub = self.create_subscription(
            ObjectsStamped,
            '/zed2/zed_node/obj_det/objects',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Detection3DArray,
            '/detections',
            10
        )

    def callback(self, msg: ObjectsStamped):
        out_msg = Detection3DArray()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id

        for obj in msg.objects:
            detection = Detection3D()
            detection.header = out_msg.header

            # Center of bounding box
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.position.z
            pose.orientation.w = 1.0
            detection.bbox.center = pose

            # Object class
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(obj.label)
            hypothesis.pose.pose.position = pose.position
            detection.results.append(hypothesis) # type: ignore

            out_msg.detections.append(detection) # type: ignore

        self.pub.publish(out_msg)
        self.get_logger().info(f'Published {len(out_msg.detections)} detections')

def main(args=None):
    rclpy.init(args=args)
    node = ZedDetectionsConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()