"""/perception/status stages and the converter's own stage."""

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
import rclpy

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


def test_relayed_accepts_foxy_level_shapes():
    packet = dict(status=[dict(name=f"n{i}", hardware_id="h", level=lv, values=[])
                          for i, lv in enumerate([2, b"\x01", [1], "2", None])])
    assert [s.level for s in perception_status.relayed(packet)] == [
        b"\x02", b"\x01", b"\x01", b"\x02", b"\x00"]


def test_relayed_skips_a_malformed_item_and_keeps_the_rest():
    packet = dict(status=[
        "junk", None, dict(name="bad", values=[dict(value="no key")]),
        dict(name="detector", hardware_id="h", values=[])])
    assert [s.name for s in perception_status.relayed(packet)] == ["perception/legacy_zed/detector"]
    assert perception_status.relayed(dict(status=None)) == []
