"""The legacy camera DDS profile must keep a large UDP receive buffer."""

from pathlib import Path
import xml.etree.ElementTree as ET

PROFILE = Path(__file__).resolve().parents[1] / "config" / "legacy_camera_dds.xml"
MIN_RECEIVE_BUFFER = 4194304


def test_camera_udp_receive_buffer_is_large_enough():
    """Off-host camera subscribers otherwise fall back to ~208 KiB sockets."""
    for descriptor in ET.parse(PROFILE).iter():
        if descriptor.tag.endswith("transport_descriptor") and any(
                child.tag.endswith("transport_id") and child.text == "camera_udp"
                for child in descriptor):
            size = next(c for c in descriptor if c.tag.endswith("receiveBufferSize"))
            assert int(size.text) >= MIN_RECEIVE_BUFFER
            return
    raise AssertionError("camera_udp transport descriptor missing")
