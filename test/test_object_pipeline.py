"""Real Jazzy message/TF tests; run in a network-isolated ROS container."""

import math
import os

import pytest

pytest.importorskip("rclpy")
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

from mhseals_nav.zed_detections_converter import convert_objects
from mhseals_nav.object_tracker import ObjectTracker
from mhseals_nav.camera_static_tf import CameraStaticTF


def packet(stamp=10):
    return dict(
        header=dict(frame_id="camera_link", stamp=dict(sec=stamp, nanosec=0)),
        objects=[
            dict(
                position=[5.0, 0.0, 0.0],
                label="red",
                confidence=90.0,
                tracking_available=True,
                tracking_state=1,
            )
        ],
    )


def test_zed_array_and_tracking_state():
    value = packet()
    converted = convert_objects(value)
    assert converted.header.frame_id == "camera_link"
    assert converted.detections[0].bbox.center.position.x == 5.0
    assert converted.detections[0].results[0].hypothesis.score == 0.9
    for state in [0, 2, 3]:
        value["objects"][0]["tracking_state"] = state
        assert not convert_objects(value).detections


def transform(parent, child, stamp, x=0.0, yaw=0.0):
    result = TransformStamped()
    result.header.frame_id, result.child_frame_id = parent, child
    result.header.stamp = Time(sec=stamp)
    result.transform.translation.x = x
    result.transform.rotation.z = math.sin(yaw / 2)
    result.transform.rotation.w = math.cos(yaw / 2)
    return result


@pytest.fixture
def tracker():
    assert os.environ.get("ROS_DOMAIN_ID") == "231", "isolate ROS tests"
    rclpy.init()
    node = ObjectTracker()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_moving_boat_uses_acquisition_tf_and_map_correction(tracker):
    # Same world buoy while boat translates 2m then rotates 90 degrees.
    tracker.tf_buffer.set_transform(transform("odom", "camera_link", 10), "test")
    tracker.tf_buffer.set_transform(
        transform("odom", "camera_link", 11, 2.0, math.pi / 2), "test"
    )
    before = convert_objects(packet(10))
    after = convert_objects(packet(11))
    after.detections[0].bbox.center.position.x = 0.0
    after.detections[0].bbox.center.position.y = -3.0
    for msg in [before, after]:
        observations = tracker.transform_detections(msg)
        assert observations[0][0] == pytest.approx([5.0, 0.0, 0.0])
        tracker.tracks.update(observations, float(msg.header.stamp.sec))
    assert len(tracker.tracks.tracks) == 1
    tracker.tracks.confirmation_count = 2
    tracker.tf_buffer.set_transform(transform("map", "odom", 11, 10.0), "test")
    arrays, markers = [], []
    tracker.object_pub = type(
        "Pub", (), {"publish": lambda _, msg: arrays.append(msg)}
    )()
    tracker.marker_pub = type(
        "Pub", (), {"publish": lambda _, msg: markers.append(msg)}
    )()
    tracker.publish_objects()
    assert arrays[-1].detections[0].bbox.center.position.x == pytest.approx(15.0)
    tracker.tf_buffer.set_transform(transform("map", "odom", 12, 20.0), "test")
    tracker.publish_objects()
    assert arrays[-1].detections[0].bbox.center.position.x == pytest.approx(25.0)
    tracker.tracks.expire(15.0)
    tracker.publish_objects()
    assert not arrays[-1].detections
    assert len(markers[-1].markers) == 2
    assert all(m.action == Marker.DELETE for m in markers[-1].markers)


def test_camera_tf_allowlist():
    from rosidl_runtime_py.convert import message_to_ordereddict

    selector = CameraStaticTF("camera_root")
    items = [
        transform("map", "odom", 0),
        transform("odom", "camera_root", 0),
        transform("camera_root", "left_camera", 0),
        transform("left_camera", "left_optical", 0),
        transform("another_camera", "other_optical", 0),
    ]
    result = selector.update(
        dict(transforms=[message_to_ordereddict(t) for t in items])
    )
    assert {t.child_frame_id for t in result} == {"left_camera", "left_optical"}


def test_missing_tf_retries_then_stale_packets_are_dropped(tracker):
    stamp = tracker.get_clock().now().to_msg()
    msg = convert_objects(packet(stamp.sec))
    msg.header.stamp = stamp
    msg.detections[0].header = msg.header
    tracker.detections_callback(msg)
    tracker.tick()
    assert len(tracker.pending) == 1
    assert not tracker.tracks.tracks
    tracker.tf_buffer.set_transform_static(transform("odom", "camera_link", 0), "test")
    tracker.tick()
    assert not tracker.pending
    assert len(tracker.tracks.tracks) == 1
    tracker.detections_callback(convert_objects(packet(1)))
    tracker.tick()
    assert not tracker.pending
    assert tracker.tracks.tracks[0].hits == 1
