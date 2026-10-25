"""/perception/status stage building."""

import pytest

pytest.importorskip("diagnostic_msgs")

from mhseals_nav import perception_status


def test_stage_clock_counts_drops_percentiles_and_objects():
    clock = perception_status.StageClock(window=10.0)
    for i in range(20):
        clock.accept(100.0 + i * .1, .05 + i * .01, objects=i % 3)
    clock.drop("stale")
    clock.drop("malformed")
    values = clock.values(102.0)
    assert values["hz"] == "2.00" and values["drop_stale"] == 1 and values["drop_reorder"] == 0
    assert int(values["latency_ms_p95"]) >= 230
    assert values["proc_ms"] == "145" and values["objects"] == "0.9"
    assert clock.values(200.0)["hz"] == "0.00"


def test_status_names_the_stage_under_its_source():
    out = perception_status.status("backend", "orin", dict(hz="15.00", ids="true"),
                                   source="zed_native")
    assert out.name == "perception/zed_native/backend" and out.hardware_id == "orin"
    assert {v.key: v.value for v in out.values} == dict(hz="15.00", ids="true")


def test_status_requires_a_source():
    with pytest.raises(TypeError):
        perception_status.status("backend", "orin", {})
