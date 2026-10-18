"""Commissioning: find a yellow buoy, circle it, and return to the start."""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import (
    Navigate, PlanCircle, PlanHome, Report, Search,
)


def build(params):
    """Stage on the near side and navigate two open arcs to complete a lap."""
    if params['circle_radius'] <= 0 or params['circle_points'] < 4:
        raise ValueError('circle_radius must be > 0 and circle_points >= 4')
    return Sequence('circle test', memory=True, children=[
        Search('find a yellow buoy', (params['mark'],),
               params['search_distance']),
        PlanCircle('plan yellow circle', 'yellow_circle', (params['mark'],),
                   params['circle_radius'], params['circle_clockwise'],
                   points=params['circle_points']),
        Navigate('approach the near side', 'yellow_circle_entry'),
        Navigate('first half of the circle', 'yellow_circle'),
        Navigate('second half of the circle', 'yellow_circle_rest'),
        Report('report lap', 'buoy_circled'),
        PlanHome('plan original pose', 'circle_home'),
        Navigate('back to the original pose', 'circle_home', precise=True),
        Report('report home', 'returned_home'),
    ])
