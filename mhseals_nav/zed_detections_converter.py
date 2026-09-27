"""Native ZED or legacy rosbridge JSON -> the local Jazzy detection contract."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


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
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(obj["label"])
        hypothesis.hypothesis.score = max(0.0, min(1.0, confidence / 100.0))
        hypothesis.pose.pose = pose
        detection.results.append(hypothesis)
        array.detections.append(detection)
    return array


class ZedDetectionsConverter(Node):
    def __init__(self):
        super().__init__("zed_detections_converter")
        self.declare_parameter("objects_topic", "/front/zed_node/obj_det/objects")
        self.declare_parameter("rosbridge_url", "")
        self.declare_parameter("camera_root_frame", "")
        topic = self.get_parameter("objects_topic").value
        url = self.get_parameter("rosbridge_url").value
        self.pub = self.create_publisher(Detection3DArray, "detections", 10)
        self.source = None
        self.static_filter = None
        if url:
            from mhseals_nav.rosbridge_source import RosbridgeSource

            root = self.get_parameter("camera_root_frame").value
            if root:
                from tf2_ros import StaticTransformBroadcaster
                from mhseals_nav.camera_static_tf import CameraStaticTF

                self.static_filter = CameraStaticTF(root)
                self.static_pub = StaticTransformBroadcaster(self)
            self.source = RosbridgeSource(url, topic, self.get_logger(), bool(root))
            self.timer = self.create_timer(0.05, self.poll)
        else:
            try:
                from zed_msgs.msg import ObjectsStamped
            except ImportError:
                from zed_interfaces.msg import ObjectsStamped
            self.sub = self.create_subscription(
                ObjectsStamped, topic, self.callback, qos_profile_sensor_data
            )

    def callback(self, msg):
        self.publish_packet(message_to_ordereddict(msg))

    def poll(self):
        if self.static_filter:
            for _ in range(100):
                packet = self.source.latest("/tf_static")
                if packet is None:
                    break
                try:
                    transforms = self.static_filter.update(packet)
                    if transforms:
                        self.static_pub.sendTransform(transforms)
                except (
                    KeyError,
                    TypeError,
                    ValueError,
                    OverflowError,
                    AssertionError,
                ) as error:
                    self.get_logger().warning(
                        f"Dropped malformed camera TF: {error}",
                        throttle_duration_sec=5.0,
                    )
        packet = self.source.latest()
        if packet is not None:
            self.publish_packet(packet)

    def publish_packet(self, packet):
        try:
            self.pub.publish(convert_objects(packet))
        except (
            KeyError,
            TypeError,
            ValueError,
            OverflowError,
            AssertionError,
        ) as error:
            self.get_logger().warning(
                f"Dropped malformed ZED objects: {error}", throttle_duration_sec=5.0
            )

    def destroy_node(self):
        if self.source:
            self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedDetectionsConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
