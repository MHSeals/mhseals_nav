"""Real Jazzy message/TF tests; run in a network-isolated ROS container."""

import math
import os
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

from mhseals_nav.zed_detections_converter import (
    ZedDetectionsConverter, convert_objects, confidence_markers, detection_markers,
)
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


def test_sdk_corners_define_box_geometry_in_the_reported_frame():
    value = packet()
    value['objects'][0]['dimensions_3d'] = [99., 99., 99.]
    value['objects'][0]['bounding_box_3d'] = dict(corners=[
        dict(kp=[x, y, z]) for x in [1., 1.4]
        for y in [-.2, .2] for z in [-.5, .7]
    ])
    box = convert_objects(value).detections[0].bbox
    assert [box.center.position.x, box.center.position.y,
            box.center.position.z] == pytest.approx([1.2, 0., .1])
    assert [box.size.x, box.size.y, box.size.z] == pytest.approx([.4, .4, 1.2])


def test_delayed_duplicate_and_reordered_packets_do_not_replay_in_rviz():
    published = []
    labels = []
    now = [100_000_000_000]
    node = SimpleNamespace(
        pub=SimpleNamespace(publish=published.append),
        marker_pub=SimpleNamespace(publish=labels.append),
        marker_count=0, last_stamp=-1, display_stamp=None, max_age=1.,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])),
    )
    ZedDetectionsConverter.publish_packet(node, packet(99))
    ZedDetectionsConverter.publish_packet(node, packet(100))
    ZedDetectionsConverter.publish_packet(node, packet(99))
    ZedDetectionsConverter.publish_packet(node, packet(100))
    ZedDetectionsConverter.publish_packet(node, packet(95))
    assert len(published) == 2
    now[0] += 2_000_000_000
    ZedDetectionsConverter.expire_display(node)
    assert not published[-1].detections
    assert labels[-1].markers[0].action == Marker.DELETE
    assert node.marker_count == 0


def test_confidence_labels_are_small_above_boxes_and_do_not_change_detections():
    value = packet()
    value['header']['frame_id'] = 'front_left_camera_optical_frame'
    value['objects'][0]['dimensions_3d'] = [.4, 1.2, .2]
    array = convert_objects(value)
    label, deleted = confidence_markers(array, previous_count=2).markers
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == '90%'
    assert label.scale.z == pytest.approx(.10)
    assert label.pose.position.y == pytest.approx(-.68)
    assert label.pose.position.x == array.detections[0].bbox.center.position.x
    assert label.header == array.header
    assert label.color.r == label.color.g == label.color.b == pytest.approx(190 / 255)
    assert label.lifetime.sec == 2
    assert array.detections[0].bbox.center.position.y == 0.
    assert deleted.action == Marker.DELETE and deleted.id == 1
    assert deleted.ns == label.ns
    # Non-optical camera frames use Z up.
    array.header.frame_id = 'camera_link'
    label = confidence_markers(array).markers[0]
    assert label.pose.position.z == pytest.approx(.18)
    assert label.pose.position.y == 0.
    array.detections.clear()
    assert all(m.action == Marker.DELETE
               for m in confidence_markers(array, previous_count=2).markers)


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
    assert arrays[-1].detections[0].results[0].hypothesis.score == pytest.approx(.9)
    # Packet gaps must not erase confirmed tracks before their tracking timeout.
    assert all(m.lifetime.sec + m.lifetime.nanosec * 1e-9 == pytest.approx(tracker.tracks.timeout)
               for m in markers[-1].markers)
    tracker.tf_buffer.set_transform(transform("map", "odom", 12, 20.0), "test")
    tracker.publish_objects()
    assert arrays[-1].detections[0].bbox.center.position.x == pytest.approx(25.0)
    tracker.tf_buffer.clear()
    count = len(markers)
    tracker.publish_objects()
    assert len(markers) == count  # missing TF is not a track deletion
    tracker.tracks.expire(11.0 + tracker.get_parameter("track_timeout").value + 1.0)
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


def test_outline_markers_use_acquisition_tf_and_delete_disappeared_boxes():
    array = convert_objects(packet())
    array.detections[0].bbox.size.x = .4
    array.detections[0].bbox.size.y = .6
    array.detections[0].bbox.size.z = 1.2
    markers = detection_markers(array, previous_count=2).markers
    box = next(m for m in markers if m.ns == "detection_boxes" and m.action == Marker.ADD)
    assert box.type == Marker.LINE_LIST
    assert len(box.points) == 24
    assert box.header == array.header  # standard Marker display hides boxes if this TF is unavailable
    assert not box.frame_locked
    assert box.pose == array.detections[0].bbox.center
    assert box.scale.x == pytest.approx(.01)
    assert sorted({p.x for p in box.points}) == pytest.approx([-.2, .2])
    assert sorted({p.z for p in box.points}) == pytest.approx([-.6, .6])
    assert {(m.ns, m.id) for m in markers if m.action == Marker.DELETE} == {
        ("confidence", 1), ("detection_boxes", 1)}
