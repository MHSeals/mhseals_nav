"""Task 6, return to home: back through the gate of two black buoys by the start."""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import GateRun, Navigate, PlanHome, Report


def build(params):
    from mhseals_nav.tasks.trees import gate_options
    black = params['marker']
    return Sequence('return home', memory=True, children=[
        PlanHome('plan home', 'home_route', params['home_approach']),
        Navigate('back toward home', 'home_route'),
        GateRun('through the home gate', 'home_gate', black, black, gates=1,
                search=params['search_distance'], max_width=params['max_gate_width'],
                near='home', radius=params['home_radius'], **gate_options(params)),
        Report('report home', 'returned_home'),
    ])
