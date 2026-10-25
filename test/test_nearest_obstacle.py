"""Nearest lethal cell from the hull, in the body frame; the costmap patches in place."""

import math
from types import SimpleNamespace

from mhseals_nav.nearest_obstacle import Grid, nearest
import numpy
import pytest

RES, HALF = 0.05, (0.5315, 0.4475)


def grid(*points, size=240):
    """Build a 12 m rolling-window grid centred on the world origin with lethal cells at points."""
    cells = numpy.zeros((size, size), numpy.int8)
    origin = (-size * RES / 2, -size * RES / 2)
    for x, y in points:
        cells[int((y - origin[1]) / RES), int((x - origin[0]) / RES)] = 100
    return cells, origin


def test_dock_beside_the_hull_reports_the_gap_toward_it():
    cells, origin = grid((0.0, -0.8))                 # 0.35 m off the starboard side
    gx, gy = nearest(cells, RES, origin, (0.0, 0.0, 0.0), HALF, 1.5)
    assert gx == pytest.approx(0.0, abs=RES) and gy == pytest.approx(-0.35, abs=RES)


def test_body_frame_follows_the_heading():
    cells, origin = grid((0.0, -0.8))
    gx, gy = nearest(cells, RES, origin, (0.0, 0.0, -math.pi / 2), HALF, 1.5)
    assert gy == pytest.approx(0.0, abs=RES) and gx == pytest.approx(0.8 - HALF[0], abs=RES)


def test_hull_clearance_picks_the_side_not_the_centre():
    # Ahead is 0.9 m from the centre (0.37 from the bow); abeam 0.8 (0.35 from the side).
    cells, origin = grid((0.9, 0.0), (0.0, 0.8), (0.0, 0.85))
    gx, gy = nearest(cells, RES, origin, (0.0, 0.0, 0.0), HALF, 1.5)
    assert gy > 0.3 and abs(gx) < RES


def test_nothing_within_radius_and_inferior_costs_are_none():
    cells, origin = grid((3.0, 0.0))
    cells[120, 125] = 99                              # inscribed inflation: not lethal
    assert nearest(cells, RES, origin, (0.0, 0.0, 0.0), HALF, 1.5) is None
    assert nearest(cells, RES, origin, (2.0, 0.0, 0.0), HALF, 1.5) is not None


def test_cell_under_the_hull_points_at_it():
    cells, origin = grid((0.3, 0.0))
    gx, gy = nearest(cells, RES, origin, (0.0, 0.0, 0.0), HALF, 1.5)
    assert 0 < math.hypot(gx, gy) <= 1e-3 and gx > 0


def test_grid_patches_updates_and_ignores_misfits():
    g = Grid()
    info = SimpleNamespace(width=4, height=3, resolution=0.15,
                           origin=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)))
    g.update(SimpleNamespace(x=0, y=0, width=1, height=1, data=[100]), 0.0)
    assert g.cells is None
    g.full(SimpleNamespace(info=info, header=SimpleNamespace(frame_id='odom'), data=[0] * 12), 1.0)
    g.update(SimpleNamespace(x=1, y=2, width=2, height=1, data=[100, 50]), 2.0)
    assert g.cells[2].tolist() == [0, 100, 50, 0] and g.received == 2.0
    g.update(SimpleNamespace(x=3, y=2, width=2, height=1, data=[1, 1]), 3.0)
    assert g.received == 2.0 and g.origin == (1.0, 2.0) and g.frame == 'odom'
