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
