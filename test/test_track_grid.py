"""Track grid: lethal discs for still tracks, a graded swath ahead of moving ones."""

from types import SimpleNamespace as NS

import numpy as np

from tracker_fixtures import CFG
from mhseals_nav.track_grid import cells, grid_origin, rasterize

GRID = {**CFG['grid'], 'radius': CFG['radius']}


def track(x, y, vx=0.0, vy=0.0, motion=1, cls='red_buoy', sigma=0.05):
    P = np.diag([sigma ** 2, sigma ** 2, 0.01, 0.01])
    return NS(class_id=cls, motion=motion, x=np.array([x, y, vx, vy]), P=P, t=0.0)


def at(grid, origin, x, y):
    res = GRID['resolution']
    return int(grid[int((y - origin[1]) // res), int((x - origin[0]) // res)])


def test_empty_grid_is_free_and_sized():
    grid = rasterize([], (3.3, -1.2), GRID)
    assert grid.shape == (cells(GRID),) * 2 == (160, 160)
    assert grid.dtype == np.int8 and not grid.any()


def test_static_disc_is_lethal_with_radius():
    centre = (0.0, 0.0)
    grid = rasterize([track(2.0, 1.0)], centre, GRID)
    origin = grid_origin(centre, GRID)
    assert at(grid, origin, 2.0, 1.0) == 100
    assert at(grid, origin, 2.0 + 0.6, 1.0) == 0


def test_moving_swath_cost_is_monotone_in_time():
    centre = (0.0, 0.0)
    grid = rasterize([track(0.0, 3.0, vx=1.0, motion=2, cls='yellow_target_boat')],
                     centre, GRID, now=0.0)
    origin = grid_origin(centre, GRID)
    costs = [at(grid, origin, x, 3.0) for x in np.arange(0.0, 3.01, 0.25)]
    assert costs[0] == 100
    assert all(0 < c <= 99 for c in costs[5:])      # past the lethal t=0 disc
    assert all(a >= b for a, b in zip(costs, costs[1:]))
    assert min(costs) >= GRID['min_cost']
    assert at(grid, origin, -2.0, 3.0) == 0          # nothing behind it


def test_latency_shifts_the_moving_disc():
    centre = (0.0, 0.0)
    origin = grid_origin(centre, GRID)
    grid = rasterize([track(0.0, 3.0, vx=1.0, motion=2, cls='yellow_target_boat')],
                     centre, GRID, now=1.0)
    assert at(grid, origin, 1.0, 3.0) == 100
    assert at(grid, origin, -0.5, 3.0) == 0


def test_origin_snaps_to_cells():
    a = grid_origin((1.01, 2.02), GRID)
    assert all(abs(v / GRID['resolution'] - round(v / GRID['resolution'])) < 1e-9 for v in a)
