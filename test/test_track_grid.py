"""Track grid: lethal discs for still tracks, a graded swath ahead of moving ones."""

from types import SimpleNamespace as NS

import numpy as np

from tracker_fixtures import CFG
from mhseals_nav.track_grid import cells, grid_origin, rasterize

# The rasterizer's geometry is checked at a wide growth cap, so rings and growth span
# several cells; the shipped cap (config/tracker.yaml) is smaller.
GRID = {**CFG['grid'], 'radius': CFG['radius'], 'max_extra_m': 0.5}


def track(x, y, vx=0.0, vy=0.0, motion=1, cls='red_buoy', sigma=0.05):
    P = np.diag([sigma ** 2, sigma ** 2, 0.01, 0.01])
    return NS(class_id=cls, motion=motion, x=np.array([x, y, vx, vy]), P=P, t=0.0,
              seen_xy=np.array([x, y]), seen_P=P[:2, :2].copy())


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


def test_unseen_static_disc_keeps_its_last_measured_size():
    centre = (0.0, 0.0)
    origin = grid_origin(centre, GRID)
    buoy = track(2.0, 1.0)
    buoy.x[:2] += 0.4                               # predicted drift while out of view
    buoy.P[:2, :2] = np.eye(2) * 1.0                # and its predicted growth
    grid = rasterize([buoy], centre, GRID)
    assert at(grid, origin, 2.0, 1.0) == 100        # still lethal where it was seen
    assert at(grid, origin, 2.0 + 0.6, 1.0) == 0    # but no wider
    unknown = rasterize([NS(**{**vars(buoy), 'motion': 0})], centre, GRID)
    assert at(unknown, origin, 2.0 + 0.6, 1.0) == 100


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


def test_only_the_moving_hull_is_lethal():
    centre = (0.0, 0.0)
    origin = grid_origin(centre, GRID)
    boat = track(0.0, 3.0, vx=0.0, vy=1.0, motion=2, cls='yellow_target_boat', sigma=0.2)
    grid = rasterize([boat], centre, GRID, now=0.0)
    hull = GRID['radius']['yellow_target_boat']
    assert at(grid, origin, hull - 0.1, 3.0) == 100
    assert at(grid, origin, hull + 0.2, 3.0) == GRID['ring_cost'] < 100
    # The same uncertainty around a buoy stays lethal.
    still = rasterize([track(0.0, 3.0, cls='yellow_target_boat', sigma=0.2)], centre, GRID)
    assert at(still, origin, hull + 0.2, 3.0) == 100


def test_stale_swath_shrinks_and_ends_with_the_horizon():
    centre = (0.0, 0.0)
    origin = grid_origin(centre, GRID)
    boat = track(0.0, 3.0, vx=1.0, motion=2, cls='yellow_target_boat')
    h = GRID['horizon_s']
    fresh = rasterize([boat], centre, GRID, now=0.0)
    stale = rasterize([boat], centre, GRID, now=h - 1.0)
    gone = rasterize([boat], centre, GRID, now=h + 0.5)
    assert at(fresh, origin, h - 0.2, 3.0) > 0
    assert at(stale, origin, h - 0.2, 3.0) > 0       # still within the last prediction
    reach = GRID['radius']['yellow_target_boat'] + GRID['max_extra_m'] + 0.2
    assert at(stale, origin, h + reach, 3.0) == 0    # nothing beyond it
    assert at(gone, origin, h + 0.5, 3.0) == 100     # the hull where it should be now
    assert at(gone, origin, h + 0.5 + reach, 3.0) == 0   # and no swath ahead


def test_origin_snaps_to_cells():
    a = grid_origin((1.01, 2.02), GRID)
    assert all(abs(v / GRID['resolution'] - round(v / GRID['resolution'])) < 1e-9 for v in a)
