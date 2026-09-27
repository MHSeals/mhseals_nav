"""Loopback WebSocket -> real Jazzy Detection3DArray, no external network."""

import json
import os
from threading import Event, Thread
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("websocket")
pytest.importorskip("websockets")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from vision_msgs.msg import Detection3DArray
from tf2_msgs.msg import TFMessage
from websockets.sync.server import serve

from mhseals_nav.zed_detections_converter import ZedDetectionsConverter


def test_legacy_json_reconnects_and_publishes_local_ros():
    assert os.environ.get("ROS_DOMAIN_ID") == "231"
    stop = Event()
    connections = []
    topic = "/zed/zed_node/obj_det/objects"

    def handler(socket):
        subscription = json.loads(socket.recv())
        assert subscription["op"] == "subscribe"
        assert subscription["topic"] == topic
        assert "type" not in subscription  # Works with zed_interfaces OR zed_msgs.
        assert json.loads(socket.recv())["topic"] == "/tf_static"
        transforms = []
        for parent, child in [
            ("zed_camera_link", "zed_left_camera_frame"),
            ("map", "odom"),
        ]:
            transforms.append(
                dict(
                    header=dict(frame_id=parent),
                    child_frame_id=child,
                    transform=dict(
                        translation=dict(x=0.0, y=0.0, z=0.0),
                        rotation=dict(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                )
            )
        socket.send(
            json.dumps(
                dict(op="publish", topic="/tf_static", msg=dict(transforms=transforms))
            )
        )
        connections.append(1)
        for _ in range(15):
            packet = dict(
                header=dict(
                    frame_id="zed_left_camera_frame",
                    stamp=dict(sec=int(time.time()), nanosec=0),
                ),
                objects=[
                    dict(
                        label="red",
                        position=[3.0, 2.0, 1.0],
                        confidence=85.0,
                        tracking_available=True,
                        tracking_state=1,
                    )
                ],
            )
            socket.send(json.dumps(dict(op="publish", topic=topic, msg=packet)))
            if stop.wait(0.05):
                break
        # Closing the first connection must cause a bounded reconnect.

    with serve(handler, "127.0.0.1", 0) as server:
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"ws://127.0.0.1:{server.socket.getsockname()[1]}"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"rosbridge_url:={url}",
                "-p",
                f"objects_topic:={topic}",
                "-p",
                "camera_root_frame:=zed_camera_link",
            ]
        )
        converter = ZedDetectionsConverter()
        probe = rclpy.create_node("bridge_test_receiver")
        received = []
        subscription = probe.create_subscription(
            Detection3DArray, "/detections", received.append, 10
        )
        static = []
        tf_subscription = probe.create_subscription(
            TFMessage,
            "/tf_static",
            static.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        executor = SingleThreadedExecutor()
        executor.add_node(converter)
        executor.add_node(probe)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and (
                len(connections) < 2 or len(received) < 3
            ):
                executor.spin_once(timeout_sec=0.05)
            assert len(connections) >= 2
            assert len(received) >= 3
            assert received[-1].header.frame_id == "zed_left_camera_frame"
            detection = received[-1].detections[0]
            assert detection.bbox.center.position.x == 3.0
            assert detection.results[0].hypothesis.class_id == "red"
            assert static
            assert {t.child_frame_id for msg in static for t in msg.transforms} == {
                "zed_left_camera_frame"
            }
        finally:
            stop.set()
            converter.destroy_node()
            probe.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            server.shutdown()
            thread.join(timeout=2)
