"""The course-layout belief (mhseals_nav/tasks/layout.py): priors, evidence and the plan."""

import math

from mhseals_nav.tasks.layout import inside, Layout, side_boxes
from mhseals_nav.tasks.tracks import Track
import numpy as np
import pytest

pytest.importorskip('py_trees')
from mhseals_nav.tasks.search import Coverage  # noqa: E402, I100

LAYOUT = {'transit_mps': 0.5, 'search_s_per_m': 2.0, 'miss': 0.2, 'exit_margin_s': 10.0,
          'exit_mps': 0.5, 'exit_factor': 1.2, 'exit_pad_m': 2.0, 'exit_floor_s': 60.0,
          'measure_m': 50.0, 'moving_mps': 0.05,
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
    short = layout.plan(['near', 'far'], (30.0, 0.0), via, 450.0, guesses, rows, 240.0)
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
    assert layout.exit_speed == 0.35 and layout.exit_factor == 1.2
    assert layout.tasks['speed_challenge']['pad'] == params['speed_challenge'][
        'circle_radius'] + layout.exit_pad_m
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


def corners(regions, side, pad):
    return [way((x, y)) for ((x0, y0), (x1, y1)), _ in side_boxes(regions, side)
            for x in (x0 - pad, x1 + pad) for y in (y0 - pad, y1 + pad)]


def test_the_reserve_covers_a_task_that_ends_in_its_regions_far_corner():
    """The reserve is sized from the region's farthest padded corner, not the guess.

    The guess-based reserve (1.5 x the way out from the guess) fell short when a task
    ended in its region's far corner: docking needed 49 m against 46 m budgeted, the
    speed task 102 m against 92 m. Now every corner of the region on the side (both
    while it is unknown), padded by the task's circle radius plus exit_pad_m, and the
    end if farther, is covered at exit_factor x the way out plus exit_margin_s.
    """
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout, grid = Layout(params), cover()
    need = {'docking': 49.0, 'speed_challenge': 102.0}
    for name in need:
        task = layout.tasks[name]
        for side in ('port', 'starboard', None):
            guess = layout.guess(name, grid, [], side)
            reserve = layout.reserve(name, guess.xy, WAY_IN, 240.0, side)
            far = max(corners(task['regions'], side, task['pad']))
            assert far > max(corners(task['regions'], side, 0.0)) >= need[name]
            assert reserve == pytest.approx(1.2 * far / 0.35 + layout.exit_margin_s)
            assert reserve > 1.5 * way(guess.xy) / layout.speed  # more than before
        # The boat's own end, farther than any corner, is covered too.
        end = (150.0, 0.0)
        assert layout.reserve(name, end, WAY_IN, 240.0, 'port') == pytest.approx(
            1.2 * way(end) / 0.35 + layout.exit_margin_s)


def test_a_mark_circled_at_the_box_edge_is_inside_the_padded_corner():
    """The speed turn circles a mark on the box edge: the way out starts past the box.

    The mark at the region's far corner, circled at circle_radius, puts the boat up to
    that radius beyond the box; the padded corner (radius plus exit_pad_m) is farther.
    """
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout = Layout(params)
    radius = params['speed_challenge']['circle_radius']
    (_, _), (x1, y1) = next(side_boxes(layout.tasks['speed_challenge']['regions'], 'port'))[0]
    reserve = layout.reserve('speed_challenge', (0.0, 0.0), WAY_IN, 240.0, 'port')
    for angle in np.linspace(0.0, 2 * math.pi, 16, endpoint=False):
        on_circle = (x1 + radius * math.cos(angle), y1 + radius * math.sin(angle))
        assert reserve > 1.2 * way(on_circle) / 0.35 + layout.exit_margin_s


def test_a_fast_mean_does_not_shrink_the_reserve_below_a_slow_real_exit():
    """Reviewer's case: measured mean 0.6 m/s, the real exit at 0.4 m/s, a 102 m corner.

    Timed at the mean the reserve would leave the speed corner 55 s short; the exit is
    timed at min(measured, exit_mps 0.35), so the slower real exit still fits.
    """
    from mhseals_nav.tasks.params import config_path, load_params
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    layout = Layout(params)
    layout.observe((0.0, 0.0, 0.0), 0.0)
    layout.observe((60.0, 0.0, 0.0), 100.0)  # 60 m in 100 s: a measured 0.6 m/s
    assert layout.speed == pytest.approx(0.6) and layout.exit_speed == 0.35
    for side in ('port', 'starboard'):
        reserve = layout.reserve('speed_challenge', (40.0, 0.0), WAY_IN, 240.0, side)
        assert reserve >= 102.0 / 0.4  # the real exit's need is covered
        # Timed at the mean it would not be: the shortfall the review found.
        assert 1.2 * 102.0 / 0.6 + layout.exit_margin_s < 102.0 / 0.4
    # A slower measured speed lowers the exit's speed further: never timed faster.
    slow = Layout(params)
    slow.moved, slow.moving = 60.0, 200.0  # 0.3 m/s measured
    assert slow.exit_speed == pytest.approx(0.3)
    assert slow.reserve('speed_challenge', (40.0, 0.0), WAY_IN, 240.0, 'port') > \
        layout.reserve('speed_challenge', (40.0, 0.0), WAY_IN, 240.0, 'port')


def test_a_task_with_no_regions_keeps_the_fixed_reserve():
    """No region to size from: the fixed exit_reserve, never the floor or the end alone."""
    layout = Layout(PARAMS)
    assert layout.reserve('none', (0.0, 20.0), [(0.0, 0.0)], 240.0) == 240.0
    bare = Layout({**PARAMS, 'near': {**PARAMS['near'], 'search': {}}})
    assert bare.reserve('near', (200.0, 0.0), [(0.0, 0.0)], 240.0) == 240.0
    # A region next to home: at least the floor.
    home = Layout({**PARAMS, 'near': {**PARAMS['near'], 'search': {
        'regions': [{'box': [[-1.0, -1.0], [1.0, 1.0]]}]}}})
    assert home.reserve('near', (0.0, 0.0), [(0.0, 0.0)], 240.0) == home.exit_floor_s
