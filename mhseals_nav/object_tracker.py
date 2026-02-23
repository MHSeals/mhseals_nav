#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.optimize import linear_sum_assignment
import hashlib
from typing import List, Optional

class PersistentObject:
    def __init__(self, position: List[float], obj_id: int, label: str, stamp: float):
        self.id = obj_id
        self.position = np.array(position, dtype=np.float32)
        self.label = label
        self.last_seen = stamp
        self.observation_count = 1

    def update(self, new_position: List[float], stamp: float):
        alpha = 0.5
        self.position = alpha * np.array(new_position) + (1 - alpha) * self.position
        self.last_seen = stamp
        self.observation_count += 1


class CandidateObject:
    def __init__(self, position: List[float], stamp: float, label: str):
        self.position = np.array(position, dtype=np.float32)
        self.label = label
        self.first_seen = stamp
        self.last_seen = stamp
        self.observation_count = 1

    def update(self, new_position: List[float], stamp: float):
        alpha = 0.5
        self.position = alpha * np.array(new_position) + (1 - alpha) * self.position
        self.last_seen = stamp
        self.observation_count += 1


class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')

        self.detection_sub = self.create_subscription(
            Detection3DArray,
            '/detections',
            self.detections_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/local',
            self.odom_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/tracked_objects',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.delay_detection_iterations = 0

        self.objects = []
        self.candidates = []

        self.next_id = 0
        self.association_distance = 3.0

        self.confirmation_time = 2.0
        self.confirmation_count = 10

        self.current_yaw_rate = 0.0
        self.yaw_rate_threshold = 0.1

    def odom_callback(self, msg: Odometry):
        self.current_yaw_rate = float(msg.twist.twist.angular.z)

    def color_from_label(self, label: str):
        h = hashlib.md5(label.encode()).hexdigest()
        r = int(h[0:2], 16) / 255.0
        g = int(h[2:4], 16) / 255.0
        b = int(h[4:6], 16) / 255.0
        return r, g, b

    def transform_to_map(self, det, frame_id: str, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose = det.bbox.center

        try:
            # Use the detection's own timestamp so the transform is interpolated
            # to the exact moment the detection was captured, preventing ghost
            # points during turns caused by TF being ahead of sensor data.
            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                stamp,  # was Time() — see comment above
                timeout=Duration(nanoseconds=int(3e8))
            )

            map_pose = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)

            return [
                float(map_pose.position.x),
                float(map_pose.position.y),
                float(map_pose.position.z)
            ]
        except Exception:
            return None

    def detections_callback(self, msg: Detection3DArray):
        if abs(self.current_yaw_rate) > self.yaw_rate_threshold:
            self.delay_detection_iterations = 2
            return

        if self.delay_detection_iterations > 0:
            self.delay_detection_iterations -= 1

        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9
        frame_id = msg.header.frame_id

        detections_map = []

        for det in msg.detections:
            pos = self.transform_to_map(det, frame_id, stamp)
            if pos is not None:
                label = "unknown"
                if len(det.results) > 0:
                    label = str(det.results[0].hypothesis.class_id)
                detections_map.append((pos, label))

        if len(detections_map) == 0:
            return

        if len(self.objects) > 0:
            cost_matrix = np.zeros((len(self.objects), len(detections_map)))

            for i, obj in enumerate(self.objects):
                for j, (det_pos, _) in enumerate(detections_map):
                    cost_matrix[i, j] = np.linalg.norm(obj.position - det_pos)

            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            assigned_detections = set()

            for obj_idx, det_idx in zip(row_ind, col_ind):
                if cost_matrix[obj_idx, det_idx] < self.association_distance:
                    det_pos, _ = detections_map[det_idx]
                    self.objects[obj_idx].update(det_pos, current_time)
                    assigned_detections.add(det_idx)
        else:
            assigned_detections = set()

        for j, (det_pos, label) in enumerate(detections_map):
            if j not in assigned_detections:
                self.handle_candidate(det_pos, current_time, label)

        self.publish_markers()

    def handle_candidate(self, position: List[float], stamp: float, label: str):
        for candidate in self.candidates:
            if np.linalg.norm(candidate.position - position) < self.association_distance:
                candidate.update(position, stamp)
                duration = candidate.last_seen - candidate.first_seen

                if duration >= self.confirmation_time:
                    if candidate.observation_count >= self.confirmation_count:
                        self.create_object(candidate.position.tolist(), stamp, candidate.label)
                    else:
                        self.candidates.remove(candidate)
                return

        self.candidates.append(CandidateObject(position, stamp, label))

    def create_object(self, position: List[float], stamp: float, label: str):
        obj = PersistentObject(position, self.next_id, label, stamp)
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


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()