"""Waypoint speed challenge: the first red or green sighting picks the route, driven with Nav2.

Routes are (forward, left) metres from the boat's pose when the choice is made,
so the same parameters work wherever the course is laid out.
"""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import Navigate, PlanRoute, Report, WaitFor


def build(params):
    routes = {label: [tuple(point) for point in route]
              for label, route in params['routes'].items()}
    return Sequence('speed challenge', memory=True, children=[
        WaitFor('signal visible', tuple(routes), any_of=True),
        PlanRoute('choose route', 'route', routes),
        Report('report choice', 'route_chosen', include=('route_choice',)),
        Navigate('drive route', 'route'),
        Report('report finish', 'route_finished'),
    ])
