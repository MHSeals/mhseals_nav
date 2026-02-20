#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
from scipy.optimize import linear_sum_assignment

class PersistentObject:
    def __init__(self, position, obj_id):
        self.id = obj_id
        self.position = np.array(position, dtype=np.float32)
        self.last_seen = 0.0
        self.observation_count = 1

    def update(self, new_position, stamp):
        # Simple running average fusion
        alpha = 0.5
        self.position = alpha * np.array(new_position) + (1 - alpha) * self.position
        self.last_seen = stamp
        self.observation_count += 1


class LongTermObjectTracker(Node):

    def __init__(self):
        super().__init__('long_term_object_tracker')

        self.subscription = self.create_subscription(
            Detection3DArray,
            '/detections',
            self.callback,
            10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.objects = []
        self.next_id = 0

        self.association_distance = 1.0  # meters

    def transform_to_map(self, det, frame_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose = det.bbox.center

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                stamp,
                timeout=Duration(seconds=1)
            )

            map_pose = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)
            return [
                map_pose.position.x,
                map_pose.position.y,
                map_pose.position.z
            ]

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None

    def callback(self, msg):

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        detections_map = []

        # Transform detections to map frame
        for det in msg.detections:
            pos = self.transform_to_map(det, frame_id, stamp)
            if pos is not None:
                detections_map.append(pos)

        if len(detections_map) == 0:
            return

        if len(self.objects) == 0:
            for det in detections_map:
                self.create_object(det, stamp)
            return

        # ----------------------------------------
        # Build cost matrix in map frame
        # ----------------------------------------
        cost_matrix = np.zeros((len(self.objects), len(detections_map)))

        for i, obj in enumerate(self.objects):
            for j, det in enumerate(detections_map):
                cost_matrix[i, j] = np.linalg.norm(obj.position - det)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        assigned_objects = set()
        assigned_detections = set()

        # Association
        for obj_idx, det_idx in zip(row_ind, col_ind):
            if cost_matrix[obj_idx, det_idx] < self.association_distance:
                self.objects[obj_idx].update(
                    detections_map[det_idx],
                    stamp.sec + stamp.nanosec * 1e-9
                )
                assigned_objects.add(obj_idx)
                assigned_detections.add(det_idx)

        # New objects
        for j, det in enumerate(detections_map):
            if j not in assigned_detections:
                self.create_object(det, stamp)

        # Debug output
        for obj in self.objects:
            self.get_logger().info(
                f'ID {obj.id} | Pos {obj.position} | Seen {obj.observation_count}'
            )

    def create_object(self, position, stamp):
        obj = PersistentObject(position, self.next_id)
        obj.last_seen = stamp.sec + stamp.nanosec * 1e-9
        self.objects.append(obj)
        self.next_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = LongTermObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()