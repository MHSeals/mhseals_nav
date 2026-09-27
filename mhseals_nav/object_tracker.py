"""Timestamped detections -> local tracks -> current map-frame objects."""

from collections import deque
import hashlib
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from mhseals_nav.object_tracks import ObjectTracks


class ObjectTracker(Node):
    """Track in odom so map->odom corrections move every landmark together."""

    def __init__(self):
        super().__init__("object_tracker")
        defaults = dict(
            tracking_frame="odom",
            map_frame="map",
            association_distance=1.0,
            track_timeout=3.0,
            candidate_timeout=1.0,
            confirmation_count=3,
            confirmation_time=0.2,
            smoothing_alpha=0.5,
            max_detection_age=1.0,
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        for key in (
            "association_distance",
            "track_timeout",
            "candidate_timeout",
            "max_detection_age",
        ):
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not (
            0 < values["smoothing_alpha"] <= 1
            and values["confirmation_count"] >= 1
            and 0 <= values["confirmation_time"] < values["track_timeout"]
        ):
            raise ValueError("invalid object confirmation/smoothing settings")
        self.tracking_frame = values["tracking_frame"]
        self.map_frame = values["map_frame"]
        self.max_age = values["max_detection_age"]
        self.tracks = ObjectTracks(
            values["association_distance"],
            values["track_timeout"],
            values["candidate_timeout"],
            values["confirmation_count"],
            values["confirmation_time"],
            values["smoothing_alpha"],
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = deque(maxlen=10)
        self.last_now = float("-inf")
        self.published_ids = set()
        self.marker_pub = self.create_publisher(MarkerArray, "tracked_objects", 10)
        self.object_pub = self.create_publisher(Detection3DArray, "objects/map", 10)
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            "detections",
            self.detections_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.tick)

    def detections_callback(self, msg):
        if len(msg.detections) <= self.tracks.max_tracks:
            self.pending.append(msg)

    def transform_detections(self, msg):
        """All-or-nothing batch; retry missing TF without blocking its listener."""
        result = []
        for det in msg.detections:
            header = det.header if det.header.frame_id else msg.header
            stamp = header.stamp
            if not header.frame_id or (stamp.sec == 0 and stamp.nanosec == 0):
                continue
            transform = self.tf_buffer.lookup_transform(
                self.tracking_frame, header.frame_id, Time.from_msg(stamp)
            )
            pose = do_transform_pose(det.bbox.center, transform)
            label = det.results[0].hypothesis.class_id if det.results else "unknown"
            result.append(
                ([pose.position.x, pose.position.y, pose.position.z], str(label))
            )
        return result

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.last_now:
            self.pending.clear()
        self.last_now = now
        self.tracks.expire(now)
        for _ in range(len(self.pending)):
            msg = self.pending.popleft()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0 or not -0.1 <= now - stamp <= self.max_age:
                continue
            try:
                observations = self.transform_detections(msg)
            except TransformException as error:
                self.pending.append(msg)
                self.get_logger().warning(
                    f"Detections waiting for TF: {error}", throttle_duration_sec=5.0
                )
                continue
            self.tracks.update(observations, stamp)
        self.publish_objects()

    def publish_objects(self):
        array = Detection3DArray()
        array.header.frame_id = self.map_frame
        array.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        visible = set()
        transform = None
        if self.tracks.objects:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.tracking_frame, Time()
                )
            except TransformException as error:
                self.get_logger().warning(
                    f"Objects waiting for map TF: {error}", throttle_duration_sec=5.0
                )
        for track in self.tracks.objects if transform is not None else []:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(
                float, track.position
            )
            pose.orientation.w = 1.0
            pose = do_transform_pose(pose, transform)
            detection = Detection3D()
            detection.header = array.header
            detection.id = str(track.id)
            detection.bbox.center = pose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = track.label
            hypothesis.pose.pose = pose
            detection.results.append(hypothesis)
            array.detections.append(detection)
            color = hashlib.md5(track.label.encode()).digest()
            for namespace, kind in [
                ("tracked_objects", Marker.SPHERE),
                ("tracked_labels", Marker.TEXT_VIEW_FACING),
            ]:
                marker = Marker()
                marker.header = array.header
                marker.ns, marker.id = namespace, track.id
                marker.type, marker.action = kind, Marker.ADD
                marker.pose.position.x = pose.position.x
                marker.pose.position.y = pose.position.y
                marker.pose.position.z = pose.position.z + (
                    0.5 if kind == Marker.TEXT_VIEW_FACING else 0.0
                )
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.3
                marker.color.r, marker.color.g, marker.color.b = (
                    component / 255.0 for component in color[:3]
                )
                marker.color.a = 1.0
                marker.text = track.label
                marker.lifetime = Duration(seconds=0.5).to_msg()
                markers.markers.append(marker)
            visible.add(track.id)
        for identifier in self.published_ids - visible:
            for namespace in ("tracked_objects", "tracked_labels"):
                marker = Marker()
                marker.header = array.header
                marker.ns, marker.id = namespace, identifier
                marker.action = Marker.DELETE
                markers.markers.append(marker)
        self.published_ids = visible
        self.object_pub.publish(array)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
