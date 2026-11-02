"""/perception/nearby: PointCloud2 xyz, sector ranges from the bow clockwise, height and hull."""

import struct
from types import SimpleNamespace

from mhseals_nav.nearby import Rate, summarize, xyz
import numpy
import pytest
import yaml

HULL = (0.6315, 0.5475)


def cloud(points, extra=4, pad=0, datatype=7):
    """A PointCloud2 stand-in: x, y, z then ``extra`` bytes per point, ``pad`` bytes per row."""
    fmt, size = ('<fff', 12) if datatype == 7 else ('<ddd', 24)
    step = size + extra
    data = b''.join(struct.pack(fmt, *p) + b'\0' * extra for p in points) + b'\0' * pad
    fields = [SimpleNamespace(name=n, offset=i * (size // 3), datatype=datatype)
              for i, n in enumerate('xyz')]
    return SimpleNamespace(fields=fields, point_step=step, width=len(points), height=1,
                           row_step=len(points) * step + pad, is_bigendian=False, data=data)


@pytest.mark.parametrize('datatype', [7, 8])
def test_xyz_reads_the_fields_and_drops_non_finite_points(datatype):
    got = xyz(cloud([(1, 2, 3), (float('nan'), 0, 0), (4, 5, 6)], pad=3, datatype=datatype))
    assert got.tolist() == [[1, 2, 3], [4, 5, 6]]


def test_xyz_without_float_xyz_is_empty():
    msg = cloud([(1, 2, 3)])
    msg.fields = msg.fields[:2]
    assert xyz(msg).shape == (0, 3)
    assert xyz(cloud([])).shape == (0, 3)


def test_xyz_of_a_malformed_cloud_is_empty():
    short_rows = cloud([(1, 2, 3), (4, 5, 6)])
    short_rows.row_step = 8
    short_data = cloud([(1, 2, 3)])
    short_data.data = short_data.data[:-1]
    past_step = cloud([(1, 2, 3)])
    past_step.fields[2].offset = 14
    for msg in (short_rows, short_data, past_step):
        assert xyz(msg).shape == (0, 3)


def test_sectors_start_at_the_bow_and_go_clockwise():
    body = numpy.array([[5.0, 0.0, 0.0],      # dead ahead: sector 0
                        [0.0, -3.0, 0.0],     # starboard beam: 90 deg, sector 4
                        [0.0, 2.0, 0.0],      # port beam: 270 deg, sector 12
                        [8.0, 0.0, 0.0]])     # farther, same sector as the first
    out = summarize(body, 16, -0.1, 3.0, HULL)
    assert out['count'] == 4
    assert out['sectors'][0] == 5.0 and out['sectors'][4] == 3.0 and out['sectors'][12] == 2.0
    assert sum(r is not None for r in out['sectors']) == 3
    assert out['nearest'] == {'range': 2.0, 'bearing': 270.0}


def test_height_band_and_hull_leave_points_out():
    body = numpy.array([[3.0, 0.0, -0.5],     # below: water and spray
                        [3.0, 0.0, 4.0],      # above
                        [0.5, 0.3, 0.2],      # on the hull
                        [0.0, 0.0, 0.0]])
    out = summarize(body, 16, -0.1, 3.0, HULL)
    assert out == {'sectors': [None] * 16, 'nearest': None, 'count': 0}


def test_just_left_of_the_bow_stays_in_sector_zero():
    out = summarize(numpy.array([[4.0, 0.5, 0.0]]), 16, -0.1, 3.0, HULL)
    assert out['sectors'][0] is not None and 352 < out['nearest']['bearing'] < 360


def test_rate_counts_arrivals_in_the_window():
    rate = Rate(window=2.0)
    assert (rate.tick(0.0), rate.hz()) == (None, 0.0)
    for i in range(1, 30):
        rate.tick(i * 0.1)
    assert rate.hz() == pytest.approx(10.0)


def test_config_section_has_the_node_keys():
    from pathlib import Path
    from mhseals_nav.yaml_params import section_config
    near = section_config('lidar.yaml', 'costmap', path=Path(__file__).parents[1] / 'config/lidar.yaml')
    assert set(near) == {'cloud_topic', 'topic', 'sectors', 'rate', 'min_height', 'max_height',
                         'half_length', 'half_width', 'hull_margin'}
    assert near['topic'] == '/perception/nearby' and near['sectors'] == 16
