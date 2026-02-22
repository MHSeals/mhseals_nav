#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.optimize import linear_sum_assignment
import hashlib


class PersistentObject:
    def __init__(self, position, obj_id, label):
        self.id = obj_id
        self.position = np.array(position, dtype=np.float32)
        self.label = label
        self.last_seen = 0.0
        self.observation_count = 1

    def update(self, new_position, stamp):
        alpha = 0.5
        self.position = alpha * np.array(new_position) + (1 - alpha) * self.position
        self.last_seen = stamp
        self.observation_count += 1


class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')

        self.subscription = self.create_subscription(
            Detection3DArray,
            '/detections',
            self.callback,
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/tracked_objects',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.objects = []
        self.next_id = 0
        self.association_distance = 1.0

    def color_from_label(self, label):
        h = hashlib.md5(label.encode()).hexdigest()
        r = int(h[0:2], 16) / 255.0
        g = int(h[2:4], 16) / 255.0
        b = int(h[4:6], 16) / 255.0
        return r, g, b

    def transform_to_map(self, det, frame_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose = det.bbox.center

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                Time(),
                timeout=Duration(seconds=1)
            )

            map_pose = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)

            return [
                map_pose.position.x,
                map_pose.position.y,
                map_pose.position.z
            ]

        except Exception:
            return None

    def callback(self, msg):

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        detections_map = []

        for det in msg.detections:
            pos = self.transform_to_map(det, frame_id, stamp)
            if pos is not None:
                label = "unknown"
                if len(det.results) > 0:
                    label = det.results[0].hypothesis.class_id
                detections_map.append((pos, label))

        if len(detections_map) == 0:
            return

        if len(self.objects) == 0:
            for pos, label in detections_map:
                self.create_object(pos, stamp, label)
            self.publish_markers()
            self.get_logger().info(f"Currently tracking {len(self.objects)} objects")
            return

        cost_matrix = np.zeros((len(self.objects), len(detections_map)))

        for i, obj in enumerate(self.objects):
            for j, (det_pos, _) in enumerate(detections_map):
                cost_matrix[i, j] = np.linalg.norm(obj.position - det_pos)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        assigned_objects = set()
        assigned_detections = set()

        for obj_idx, det_idx in zip(row_ind, col_ind):
            det_pos, _ = detections_map[det_idx]
            if cost_matrix[obj_idx, det_idx] < self.association_distance:
                self.objects[obj_idx].update(
                    det_pos,
                    stamp.sec + stamp.nanosec * 1e-9
                )
                assigned_objects.add(obj_idx)
                assigned_detections.add(det_idx)

        for j, (det_pos, label) in enumerate(detections_map):
            if j not in assigned_detections:
                self.create_object(det_pos, stamp, label)

        self.publish_markers()
        self.get_logger().info(f"Currently tracking {len(self.objects)} objects")

    def create_object(self, position, stamp, label):
        obj = PersistentObject(position, self.next_id, label)
        obj.last_seen = stamp.sec + stamp.nanosec * 1e-9
        self.objects.append(obj)
        self.next_id += 1

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for obj in self.objects:
            r, g, b = self.color_from_label(obj.label)

            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = now
            sphere.ns = "tracked_objects"
            sphere.id = obj.id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(obj.position[0])
            sphere.pose.position.y = float(obj.position[1])
            sphere.pose.position.z = float(obj.position[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = now
            text.ns = "tracked_labels"
            text.id = obj.id + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(obj.position[0])
            text.pose.position.y = float(obj.position[1])
            text.pose.position.z = float(obj.position[2] + 0.5)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.4
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = obj.label

            marker_array.markers.append(sphere)  # type: ignore
            marker_array.markers.append(text)  # type: ignore

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()