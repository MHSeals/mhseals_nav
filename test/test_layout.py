"""The course-layout belief (mhseals_nav/tasks/layout.py): priors, evidence and the plan."""

import math

from mhseals_nav.tasks.layout import inside, Layout
from mhseals_nav.tasks.tracks import Track
import numpy as np
import pytest

pytest.importorskip('py_trees')
from mhseals_nav.tasks.search import Coverage  # noqa: E402

PARAMS = {
    'layout': {'transit_mps': 0.5, 'search_s_per_m': 2.0, 'miss': 0.2, 'exit_margin': 1.5,
               'exit_floor_s': 60.0,
               'tasks': {'far': {'labels': ['yellow_buoy'], 'service_s': 100.0},
                         'near': {'labels': ['banner_1'], 'service_s': 50.0}}},
    'far': {'search': {'regions': [{'box': [[20.0, 10.0], [60.0, 50.0]], 'mirror': True}]}},
    'near': {'search': {'regions': [{'box': [[-10.0, 5.0], [10.0, 25.0]], 'mirror': True}]}},
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
    rows = {'far': {'value': 600.0, 'expected_s': 0.0}, 'near': {'value': 450.0, 'expected_s': 0.0},
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
