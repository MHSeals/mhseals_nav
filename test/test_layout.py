"""The course-layout belief (mhseals_nav/tasks/layout.py): priors, evidence and the plan."""

import math

from mhseals_nav.tasks.layout import inside, Layout, side_boxes
from mhseals_nav.tasks.tracks import Track
import numpy as np
import pytest

pytest.importorskip('py_trees')
from mhseals_nav.tasks.search import Coverage  # noqa: E402, I100

LAYOUT = {'transit_mps': 0.5, 'search_s_per_m': 2.0, 'miss': 0.2, 'exit_margin_s': 10.0,
          'exit_mps': 0.5, 'exit_floor_s': 60.0, 'measure_m': 50.0, 'moving_mps': 0.05,
          'tasks': {'far': ['mark'], 'near': ['landmarks']}}
PARAMS = {
    'layout': LAYOUT,
    'far': {'mark': 'yellow_buoy', 'service_s': 100.0,
            'search': {'regions': [{'box': [[20.0, 10.0], [60.0, 50.0]], 'mirror': True}]}},
    'near': {'landmarks': ['banner_1'], 'service_s': 50.0,
             'search': {'regions': [{'box': [[-10.0, 5.0], [10.0, 25.0]], 'mirror': True}]}},
}


def cover():
    return Coverage([[-20.0, -60.0], [80.0, 60.0]], 1.5)


def test_the_guess_is_the_prior_on_the_known_side_until_the_task_is_seen():
    layout, grid = Layout(PARAMS), cover()
    both = layout.guess('far', grid, [], None)
    assert both.xy[1] == pytest.approx(0.0, abs=1.0) and not both.found
    port = layout.guess('far', grid, [], 'port')
    assert port.xy == pytest.approx((40.0, 30.0), abs=1.0) and port.spread < both.spread
    seen = [Track('yellow_buoy', 50.0, 40.0, 0.9, None, True, False, '1'),
            Track('yellow_buoy', 5.0, 27.0, 0.9, None, True, False, '2')]  # a look-alike
    found = layout.guess('far', grid, seen, 'port')
    assert found.found and found.xy == pytest.approx((50.0, 40.0)) and found.spread == 0.0
    assert layout.guess('nowhere', grid, [], 'port') is None
    assert inside((5.0, -20.0), PARAMS['near']['search']['regions'], None)


def test_looking_without_a_find_moves_the_guess_to_what_is_unseen():
    layout, grid = Layout(PARAMS), cover()
    before = layout.guess('far', grid, [], 'port')
    grid.observe((20.0, 10.0, math.radians(45.0)), (math.radians(110.0), 20.0), 1.0)
    after = layout.guess('far', grid, [], 'port')
    assert math.dist(after.xy, (20.0, 10.0)) > math.dist(before.xy, (20.0, 10.0))
    assert np.isclose(layout.grid('far', grid, 'port').max(), 1.0)


def test_the_plan_fits_the_most_points_and_ends_near_the_way_out():
    layout, grid = Layout(PARAMS), cover()
    guesses = {n: layout.guess(n, grid, [], 'port') for n in ('far', 'near')}
    rows = {'far': {'value': 600.0, 'expected_s': 0.0},
            'near': {'value': 450.0, 'expected_s': 0.0},
            'off': {'value': 0.0, 'expected_s': 1.0}}
    via = [(20.0, 0.0), (0.0, 0.0)]
    plan = layout.plan(['near', 'far', 'off'], (30.0, 0.0), via, 2000.0, guesses, rows, 240.0)
    assert list(plan) == ['far', 'near']  # the marina, by the way out, last
    seconds, reserve = plan['near']
    assert 60.0 <= reserve < 240.0
    short = layout.plan(['near', 'far'], (30.0, 0.0), via, 400.0, guesses, rows, 240.0)
    assert list(short) == ['far']  # time for one: the one worth more
    assert layout.plan(['near', 'far'], (30.0, 0.0), via, 50.0, guesses, rows, 240.0) == {}


def test_the_cost_scales_with_the_speed_the_boat_is_measured_at():
    layout, grid = Layout(PARAMS), cover()
    guess = layout.guess('near', grid, [], 'port')
    slow = layout.cost('near', (40.0, 0.0), guess, 0.0)[0]
    for k in range(200):  # 1 m/s for 100 m
        layout.observe((float(k) * 0.5, 0.0, 0.0), k * 0.5)
    assert layout.speed == pytest.approx(1.0)
    assert layout.cost('near', (40.0, 0.0), guess, 0.0)[0] == pytest.approx(slow / 2.0)
    assert layout.cost('none', (1.0, 2.0), None, 99.0) == (99.0, (1.0, 2.0))


def test_the_belief_and_the_search_share_one_prior():
    layout, grid = Layout(PARAMS), cover()
    regions = PARAMS['far']['search']['regions']
    for side in (None, 'port', 'starboard'):
        assert np.array_equal(layout.grid('far', grid, side), grid.prior(regions, side, 0.0))
    assert grid.region_area(regions) is None  # mirrored: the side first
    (x0, y0), (x1, y1) = grid.region_area(regions, 'starboard')
    assert (x0, y0, x1, y1) == (20.0, -50.0, 60.0, -10.0)
    (view,) = grid.region_views(regions, (0.0, 0.0, 0.0), 10.0, 'starboard')
    assert view[1] < 0
    assert len(grid.region_views(regions, (0.0, 0.0, 0.0), 10.0)) == 2


def test_a_mirrored_box_drawn_on_starboard_gives_the_port_box_for_port():
    """The side picks the box by where it lies, not by which of the pair was drawn."""
    grid = cover()
    drawn = [{'box': [[20.0, -30.0], [60.0, -6.0]], 'mirror': True}]
    ported = [{'box': [[20.0, 6.0], [60.0, 30.0]], 'mirror': True}]
    (x0, y0), (x1, y1) = grid.region_area(drawn, 'port')
    assert (x0, y0, x1, y1) == (20.0, 6.0, 60.0, 30.0)
    (view,) = grid.region_views(drawn, (0.0, 0.0, 0.0), 10.0, 'port')
    assert view[1] > 0
    for side in ('port', 'starboard'):
        assert np.array_equal(grid.prior(drawn, side, 0.0), grid.prior(ported, side, 0.0))
    port = grid.prior(drawn, 'port', 0.0)
    assert port[np.ix_((grid.xs > 25.0) & (grid.xs < 55.0),
                       (grid.ys > 10.0) & (grid.ys < 25.0))].min() == 1.0
    assert port[:, grid.ys < 0.0].max() == 0.0


def test_the_mission_yaml_is_the_only_source_and_finals_keep_the_fixed_reserve():
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout = Layout(params)
    assert layout.tasks['docking']['labels'] == tuple(params['docking']['landmarks'])
    assert layout.tasks['speed_challenge']['labels'] == (
        'yellow_buoy', *params['speed_challenge']['indicators'])
    # Qualifying: the dynamic reserve, at least the floor near home.
    assert layout.reserve('none', (0.0, 20.0), [(0.0, 0.0)], 240.0) == layout.exit_floor_s
    with pytest.raises(KeyError):
        Layout({**params, 'layout': {k: v for k, v in params['layout'].items()
                                     if k != 'miss'}})
    finals = Layout(load_params('mission', config_path('mission_2026.yaml'), 'finals'))
    assert finals.exit_margin_s is None
    assert finals.reserve('docking', (200.0, 200.0), [(0.0, 0.0)], 300.0) == 300.0
    assert finals.reserve('docking', (0.0, 1.0), [(0.0, 0.0)], 300.0) == 300.0


WAY_IN = [(25.0, 0.0), (10.0, 0.0), (0.0, 0.0)]  # the entry gates, last first, then home


def way(xy):
    return sum(math.dist(a, b) for a, b in zip([xy, *WAY_IN], WAY_IN))


def test_the_reserve_covers_a_task_that_ends_in_its_regions_far_corner():
    """The reserve is sized from the region's farthest corner, not the guess.

    The guess-based reserve (1.5 x the way out from the guess) fell short when a task
    ended in its region's far corner: docking needed 49 m against 46 m budgeted, the
    speed task 102 m against 92 m (about 292 s at 0.35 m/s). Now every corner of the
    region on the side (both while it is unknown), and the end if farther, is covered.
    """
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout, grid = Layout(params), cover()
    need = {'docking': 49.0, 'speed_challenge': 102.0}
    for name in need:
        regions = layout.tasks[name]['regions']
        for side in ('port', 'starboard', None):
            guess = layout.guess(name, grid, [], side)
            reserve = layout.reserve(name, guess.xy, WAY_IN, 240.0, side)
            corners = [way((x, y)) for ((x0, y0), (x1, y1)), _ in side_boxes(regions, side)
                       for x in (x0, x1) for y in (y0, y1)]
            assert max(corners) >= need[name]  # the corner shortfall case is in range
            assert reserve >= max(corners) / layout.speed + layout.exit_margin_s
            assert reserve >= need[name] / 0.35 + layout.exit_margin_s, (name, side)
            assert reserve > 1.5 * way(guess.xy) / layout.speed  # more than before
        # The boat's own end, farther than any corner, is covered too.
        far = (150.0, 0.0)
        assert layout.reserve(name, far, WAY_IN, 240.0, 'port') == pytest.approx(
            way(far) / layout.speed + layout.exit_margin_s)
