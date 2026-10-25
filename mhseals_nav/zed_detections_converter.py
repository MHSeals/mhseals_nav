"""Native ZED objects -> the local Jazzy detection contract."""

import math
from copy import deepcopy
from itertools import product
from socket import gethostname
from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from mhseals_nav import perception_status
from mhseals_nav.utils.live_params import LiveParams, positive


def convert_objects(message):
    """Preserve acquisition frame/time; never invent a camera frame or TF."""
    array = Detection3DArray()
    header = message["header"]
    frame = header["frame_id"]
    if not isinstance(frame, str) or not frame or len(frame) > 256:
        raise ValueError("ZED header must name its actual coordinate frame")
    array.header.frame_id = frame
    array.header.stamp.sec = int(header["stamp"]["sec"])
    array.header.stamp.nanosec = int(header["stamp"]["nanosec"])
    if not 0 <= array.header.stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid ZED timestamp")
    objects = message["objects"]
    if not isinstance(objects, list) or len(objects) > 256:
        raise ValueError("expected at most 256 ZED objects")
    for obj in objects:
        # SEARCHING and TERMINATE are not fresh measured observations.
        if obj.get("tracking_available", False) and obj.get("tracking_state") != 1:
            continue
        position = [float(v) for v in obj["position"]]
        confidence = float(obj.get("confidence", 0.0))
        if len(position) != 3 or not all(map(math.isfinite, position + [confidence])):
            continue
        detection = Detection3D()
        detection.header = array.header
        detection.id = str(obj.get("label_id", ""))
        pose = detection.bbox.center
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        size = obj.get("dimensions_3d", [0.0, 0.0, 0.0])
        if len(size) == 3 and all(
            math.isfinite(float(v)) and float(v) >= 0 for v in size
        ):
            detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = map(
                float, size
            )
        corners = obj.get("bounding_box_3d", {}).get("corners", [])
        if len(corners) == 8:
            points = [[float(v) for v in corner["kp"]] for corner in corners]
            if any(len(p) != 3 or not all(map(math.isfinite, p)) for p in points):
                continue
            low = [min(p[i] for p in points) for i in range(3)]
            high = [max(p[i] for p in points) for i in range(3)]
            # SDK corners are already in the reported ROS frame. Enclose them
            # there, rather than assuming width/height/depth match its XYZ axes.
            if high != low:
                pose.position.x, pose.position.y, pose.position.z = (
                    (a + b) / 2 for a, b in zip(low, high)
                )
                detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = (
                    b - a for a, b in zip(low, high)
                )
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(obj["label"])
        hypothesis.hypothesis.score = max(0.0, min(1.0, confidence / 100.0))
        hypothesis.pose.pose = pose
        detection.results.append(hypothesis)
        array.detections.append(detection)
    return array


def confidence_markers(detections, previous_count=0):
    """Small labels above boxes, matching the RViz detection plugin's palette."""
    markers = MarkerArray()
    colors = {"car": (255, 165, 0), "person": (0, 0, 255),
              "cyclist": (255, 255, 0), "motorcycle": (230, 230, 250)}
    for index, detection in enumerate(detections.detections):
        if not detection.results:
            continue
        best = max(detection.results, key=lambda r: r.hypothesis.score).hypothesis
        marker = Marker()
        marker.header = detections.header
        marker.ns, marker.id = "confidence", index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position = deepcopy(detection.bbox.center.position)
        marker.pose.orientation.w = 1.0
        if "optical" in detections.header.frame_id:
            marker.pose.position.y -= detection.bbox.size.y / 2 + .08
        else:
            marker.pose.position.z += detection.bbox.size.z / 2 + .08
        marker.text = f"{best.score:.0%}"
        marker.scale.z = .10
        rgb = colors.get(best.class_id.lower(), (190, 190, 190))
        marker.color.r, marker.color.g, marker.color.b = (v / 255 for v in rgb)
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        markers.markers.append(marker)
    for index in range(len(detections.detections), previous_count):
        marker = Marker()
        marker.ns, marker.id, marker.action = "confidence", index, Marker.DELETE
        markers.markers.append(marker)
    return markers


def detection_markers(detections, previous_count=0):
    """Standard RViz markers hide on missing TF instead of drawing at the origin."""
    markers = confidence_markers(detections, previous_count)
    for index, detection in enumerate(detections.detections):
        label = markers.markers[index]
        box = Marker()
        box.header = detections.header
        box.ns, box.id = "detection_boxes", index
        box.type = Marker.LINE_LIST
        box.pose = detection.bbox.center
        box.scale.x = .01
        box.color = label.color
        box.lifetime = label.lifetime
        size = detection.bbox.size
        corners = list(product(*[(-v / 2, v / 2) for v in (size.x, size.y, size.z)]))
        for i, a in enumerate(corners):
            for b in corners[i + 1:]:
                if sum(x != y for x, y in zip(a, b)) == 1:
                    box.points.extend([Point(x=a[0], y=a[1], z=a[2]),
                                       Point(x=b[0], y=b[1], z=b[2])])
        markers.markers.append(box)
    for index in range(len(detections.detections), previous_count):
        marker = Marker()
        marker.ns, marker.id, marker.action = "detection_boxes", index, Marker.DELETE
        markers.markers.append(marker)
    return markers


class ZedDetectionsConverter(Node):
    def __init__(self):
        super().__init__("zed_detections_converter")
        self.declare_parameter("objects_topic", "/front/zed_node/obj_det/objects")
        # Stage namespace under /perception/status.
        self.declare_parameter("perception_source", "zed_native")
        self.declare_parameter("max_detection_age", 1.0)
        self.live = LiveParams(
            self, {"max_detection_age": positive},
            lambda values, changed: setattr(self, "max_age", values["max_detection_age"]))
        self.max_age = self.live["max_detection_age"]
        self.pub = self.create_publisher(Detection3DArray, "detections", 1)
        self.last_stamp = -1
        self.display_stamp = None
        self.expire_timer = self.create_timer(0.1, self.expire_display)
        self.stage = perception_status.StageClock()
        self.perception_source = self.get_parameter("perception_source").value
        self.status_pub = self.create_publisher(DiagnosticArray, "/perception/status", 1)
        self.report_timer = self.create_timer(1.0, self.report)
        try:
            from zed_msgs.msg import ObjectsStamped
        except ImportError:
            from zed_interfaces.msg import ObjectsStamped
        self.sub = self.create_subscription(
            ObjectsStamped, self.get_parameter("objects_topic").value, self.callback,
            qos_profile_sensor_data)

    def callback(self, msg):
        self.publish_packet(message_to_ordereddict(msg))

    def report(self):
        """1 Hz /perception/status with the converter stage."""
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [perception_status.status(
            "converter", gethostname(), self.stage.values(monotonic()),
            source=self.perception_source)]
        self.status_pub.publish(array)

    def publish_packet(self, packet):
        try:
            detections = convert_objects(packet)
            stamp = detections.header.stamp.sec * 1_000_000_000 + detections.header.stamp.nanosec
            now = self.get_clock().now().nanoseconds
            if stamp <= self.last_stamp:
                self.stage.drop("reorder")
                return
            if not -.1 <= (now - stamp) * 1e-9 <= self.max_age:
                self.stage.drop("stale")
                return
            self.stage.accept(monotonic(), (now - stamp) * 1e-9)
            self.last_stamp = stamp
            self.display_stamp = now
            self.last_header = detections.header
            self.pub.publish(detections)
        except (
            KeyError,
            TypeError,
            ValueError,
            OverflowError,
            AssertionError,
        ) as error:
            self.stage.drop("malformed")
            self.get_logger().warning(
                f"Dropped malformed ZED objects: {error}", throttle_duration_sec=5.0
            )

    def expire_display(self):
        if self.display_stamp is None:
            return
        age = (self.get_clock().now().nanoseconds - self.display_stamp) * 1e-9
        if -.1 <= age <= self.max_age:
            return
        empty = Detection3DArray()
        empty.header = self.last_header
        self.pub.publish(empty)
        self.display_stamp = None
        if age < 0:
            self.last_stamp = -1


def main(args=None):
    rclpy.init(args=args)
    node = ZedDetectionsConverter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
