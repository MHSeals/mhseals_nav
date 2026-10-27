"""The course-layout belief (mhseals_nav/tasks/layout.py): priors, evidence and the plan."""

import math

from mhseals_nav.tasks.layout import inside, Layout
from mhseals_nav.tasks.tracks import Track
import numpy as np
import pytest

pytest.importorskip('py_trees')
from mhseals_nav.tasks.search import Coverage  # noqa: E402, I100

LAYOUT = {'transit_mps': 0.5, 'search_s_per_m': 2.0, 'miss': 0.2, 'exit_margin': 1.5,
          'exit_floor_s': 60.0, 'measure_m': 50.0, 'moving_mps': 0.05,
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


def test_the_mission_yaml_is_the_only_source_and_both_profiles_keep_the_fixed_reserve():
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout = Layout(params)
    assert layout.tasks['docking']['labels'] == tuple(params['docking']['landmarks'])
    assert layout.tasks['speed_challenge']['labels'] == (
        'yellow_buoy', *params['speed_challenge']['indicators'])
    assert layout.exit_margin is None  # the dynamic reserve is off until Unity backs it
    assert layout.reserve((0.0, 20.0), [(0.0, 0.0)], 240.0) == 240.0
    with pytest.raises(KeyError):
        Layout({**params, 'layout': {k: v for k, v in params['layout'].items()
                                     if k != 'miss'}})
    finals = Layout(load_params('mission', config_path('mission_2026.yaml'), 'finals'))
    assert finals.exit_margin is None
    assert finals.reserve((200.0, 200.0), [(0.0, 0.0)], 300.0) == 300.0
    assert finals.reserve((0.0, 1.0), [(0.0, 0.0)], 300.0) == 300.0


def test_the_reserve_covers_a_task_that_ends_a_spread_beyond_its_guess():
    """The dynamic reserve (off by default) is planned from the guess.

    It covers a task that ends a spread beyond its guess, but not one that ends in its
    region's far corner: docking needs 49 m against the 46 m budgeted, the speed task
    102 m against 92 m (about 292 s at 0.35 m/s). That shortfall is why exit_margin is
    null in mission_2026.yaml until Unity runs show the dynamic reserve is safe.
    """
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    params = {**params, 'layout': {**params['layout'], 'exit_margin': 1.5}}
    layout, grid = Layout(params), cover()
    via = [(25.0, 0.0), (10.0, 0.0), (0.0, 0.0)]  # the entry gates, last first, then home
    for name in ('docking', 'speed_challenge'):
        for side in ('port', 'starboard'):
            guess = layout.guess(name, grid, [], side)
            out = math.dist(guess.xy, via[0])
            end = tuple(np.add(via[0], np.subtract(guess.xy, via[0])
                               * (out + guess.spread) / out))
            way = sum(math.dist(a, b) for a, b in zip([end, *via], via))
            reserve = layout.reserve(guess.xy, via, math.inf)
            assert reserve >= way / layout.speed, (name, side, reserve, way)
