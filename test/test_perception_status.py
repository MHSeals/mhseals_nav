"""Converter relay: /perception/status stages from a faked source."""

import os
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor

from mhseals_nav import perception_status


def test_relayed_keeps_stereolabs_and_prefixes_names():
    packet = dict(status=[
        dict(name="front: Camera", hardware_id="Stereolabs ZED 2i", level=1, message="slow",
             values=[dict(key="Capture Mean Frequency", value="14.8 Hz"),
                     dict(key="Capture Mean Frequency", value="x")]),
        dict(name="/battery", hardware_id="bms", level=0, values=[]),
        dict(name="perception/legacy_zed/detector", hardware_id="squirtle", level=0, values=[]),
    ])
    out = perception_status.relayed(packet, zed_only=True)
    assert [s.name for s in out] == ["perception/legacy_zed/front: Camera"]
    assert len(perception_status.relayed(packet)) == 3
    assert out[0].level == b"\x01" and len(out[0].values) == 2


def test_warnings_filters_by_level():
    logs = [dict(level=20, msg="info"), dict(level=30, msg="stale image"), dict(level=40, msg="TensorRT")]
    assert perception_status.warnings(logs) == ["stale image", "TensorRT"]


def test_stage_clock_counts_drops_and_percentiles():
    clock = perception_status.StageClock(window=10.0)
    for i in range(20):
        clock.accept(100.0 + i * .1, .05 + i * .01)
    clock.drop("stale")
    clock.drop("malformed")
    values = clock.values(102.0)
    assert values["hz"] == "2.00" and values["drop_stale"] == 1 and values["drop_reorder"] == 0
    assert int(values["latency_ms_p95"]) >= 230
    assert clock.values(200.0)["hz"] == "0.00"


class FakeSource:
    url = "ws://192.168.0.153:9090"
    topic = "/objects"
    connected = True
    packets = 0
    remote = dict(clock_offset_ms=12.34, clock_rtt_ms=5.0, remote_nodes=["/a", "/b"])

    def __init__(self, *args, **kwargs):
        self.queues = {"/objects": [], "/perception/status": [], "/diagnostics": [], "/rosout": []}

    def latest(self, topic=None):
        queue = self.queues[topic or "/objects"]
        return queue.pop(0) if queue else None

    def close(self):
        pass


class Recorder:
    def __init__(self):
        self.lines = []

    def warning(self, text, **_):
        self.lines.append(text)

    info = error = warning


def zed_packet(now, age):
    stamp = now - age
    return dict(header=dict(frame_id="cam", stamp=dict(sec=int(stamp), nanosec=int(stamp % 1 * 1e9))),
                objects=[])


def test_converter_reports_stages_and_relogs_remote_warnings(monkeypatch):
    assert os.environ.get("ROS_DOMAIN_ID") == "231", "isolate ROS tests"
    monkeypatch.setattr("mhseals_nav.rosbridge_source.RosbridgeSource", FakeSource)
    from mhseals_nav.zed_detections_converter import ZedDetectionsConverter

    rclpy.init(args=["--ros-args", "-p", "rosbridge_url:=ws://192.168.0.153:9090"])
    node = ZedDetectionsConverter()
    listener = rclpy.create_node("status_listener")
    got = []
    listener.create_subscription(DiagnosticArray, "/perception/status", got.append, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    log = Recorder()
    node.get_logger = lambda: log
    try:
        now = time.time()
        queues = node.source.queues
        queues["/objects"] += [zed_packet(now, 30.0), zed_packet(now, .1), dict(header=1)]
        queues["/perception/status"].append(dict(status=[
            dict(name="detector", hardware_id="squirtle", level=0, values=[dict(key="hz", value="4")])]))
        queues["/diagnostics"].append(dict(status=[
            dict(name="front: ZED", hardware_id="Stereolabs ZED 2i", level=0, values=[]),
            dict(name="other", hardware_id="x", level=0, values=[])]))
        queues["/rosout"] += [dict(level=30, msg="stale image"), dict(level=30, msg="stale image"),
                              dict(level=20, msg="chatter")]
        for _ in range(3):  # one packet per poll
            node.poll()
        # Malformed remote JSON must not escape the timer callback.
        queues["/diagnostics"] += [dict(status=[dict(name="bad", level=999)]), dict(status=7), ["x"]]
        queues["/rosout"].append(dict(level=30, msg="x"))
        for _ in range(3):
            node.poll()
        node.logged["old"] = time.monotonic() - 60
        node.collect_remote()
        assert "old" not in node.logged
        node.report()
        node.report()  # remote topics are throttled to 1 s: stages must survive the next report
        end = time.time() + 3
        while len(got) < 2 and time.time() < end:
            executor.spin_once(timeout_sec=.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()

    assert {s.name for s in got[1].status} == {s.name for s in got[0].status}
    stages = {s.name: {v.key: v.value for v in s.values} for s in got[0].status}
    assert set(stages) == {"perception/legacy_zed/bridge", "perception/legacy_zed/converter",
                           "perception/legacy_zed/detector", "perception/legacy_zed/front: ZED"}
    bridge = stages["perception/legacy_zed/bridge"]
    assert bridge["connected"] == "true" and bridge["peer"] == "192.168.0.153:9090"
    assert bridge["clock_offset_ms"] == "12.3" and bridge["remote_nodes"] == "/a,/b"
    converter = stages["perception/legacy_zed/converter"]
    assert converter["drop_stale"] == "1" and converter["drop_malformed"] == "1"
    assert 50 <= int(converter["latency_ms"]) <= 400
    assert log.lines.count("squirtle: stale image") == 1


def test_relayed_follows_the_source_namespace():
    packet = dict(status=[
        dict(name="perception/legacy_zed/detector", hardware_id="squirtle", level=0, values=[]),
        dict(name="front: ZED", hardware_id="Stereolabs ZED 2i", level=0, values=[]),
    ])
    out = perception_status.relayed(packet, source="bench")
    assert [s.name for s in out] == ["perception/bench/detector", "perception/bench/front: ZED"]


def test_detector_stage_reports_one_window_at_a_time():
    stage = perception_status.DetectorStage(start=100.0)
    stage.accept(.10, .20, 4, 2)
    stage.accept(.30, .40, 4, 0)
    stage.drop("stale")
    values = stage.values(101.0)
    assert values == dict(hz="2.00", drop_stale=1, proc_ms="200", proc_ms_max="300",
                          latency_ms="400", views=4, objects="1.0")
    # The window restarts; drops stay cumulative.
    assert stage.values(102.0) == dict(hz="0.00", drop_stale=1)


def test_detector_report_publishes_the_prefixed_stage():
    from mhseals_nav.legacy_buoy_detector import LegacyBuoyDetector, view_count

    assert view_count(1080, 1920, 0, 3) == 1 and view_count(1080, 1920, 720, 3) == 4
    published = []
    detector = SimpleNamespace(
        node=SimpleNamespace(get_clock=lambda: rclpy.clock.Clock()),
        perception_status=perception_status, diagnostic_type=DiagnosticArray, host="squirtle",
        stage=perception_status.DetectorStage(0.0),
        status_pub=SimpleNamespace(publish=published.append))
    LegacyBuoyDetector.report(detector)
    assert published and len(published) == 1
    (stage,) = published[0].status
    assert stage.name == "perception/legacy_zed/detector" and stage.hardware_id == "squirtle"
    assert {v.key: v.value for v in stage.values}["hz"] == "0.00"


def test_native_converter_reports_its_converter_stage():
    from mhseals_nav.zed_detections_converter import ZedDetectionsConverter

    rclpy.init()
    node = ZedDetectionsConverter()
    got = []
    node.status_pub = SimpleNamespace(publish=got.append)
    try:
        # Status goes out on the 1 Hz timer only, never per packet.
        assert node.report_timer.timer_period_ns == 1_000_000_000
        node.publish_packet(dict(header=1))
        assert got == []
        node.stage.accept(time.monotonic(), .05)
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (stage,) = got[0].status
    assert stage.name == "perception/zed_native/converter"
    assert {v.key: v.value for v in stage.values}["latency_ms"] == "50"
