import struct
from types import SimpleNamespace

from mhseals_nav.lidar_relay import decimate


class Cloud(SimpleNamespace):
    pass


def test_decimation_preserves_all_interleaved_vlp16_rings():
    cloud = Cloud(header=object(), height=1, width=1600, point_step=22,
                  row_step=1600*22, fields=object(), is_bigendian=False, is_dense=True,
                  data=b''.join(struct.pack('<H', i % 16)+bytes(20) for i in range(1600)))
    reduced = decimate(cloud, 80*22)
    rings = {struct.unpack_from('<H', reduced.data, i*22)[0] for i in range(reduced.width)}
    assert rings == set(range(16))
    assert len(reduced.data) <= 80*22
    assert reduced.header is cloud.header and reduced.fields is cloud.fields
    assert reduced.row_step == reduced.width*reduced.point_step


def test_reader_keeps_latest_scan_without_reliable_network_backpressure():
    from rclpy.qos import ReliabilityPolicy
    from mhseals_nav.lidar_relay import _qos

    assert _qos(False).depth == 1
    assert _qos(False).reliability == ReliabilityPolicy.BEST_EFFORT
    assert _qos().depth == 1
    assert _qos().reliability == ReliabilityPolicy.RELIABLE
