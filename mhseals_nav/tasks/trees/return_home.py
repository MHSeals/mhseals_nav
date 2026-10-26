"""Return home: back out through the gate by the start.

2025: the gate of two black buoys (``marker``). 2026 (``exit_source`` set): the
entry gates the boat cleared at the start, last first and the other way; if that
key is empty or the route fails, the gates are found again as red/green pairs.
"""

from mhseals_nav.tasks.behaviours import GateRun, Navigate, PlanChainBack, PlanHome, Report
from py_trees.composites import Selector, Sequence


def build(params):
    from mhseals_nav.tasks.trees import gate_options
    red, green = params.get('red', params['marker']), params.get('green', params['marker'])
    search = Sequence('find the home gate', memory=True, children=[
        PlanHome('plan home', 'home_route', params['home_approach']),
        Navigate('back toward home', 'home_route'),
        GateRun('through the home gate', 'home_gate', red, green,
                gates=params.get('gates', 1), red_side=params.get('red_side', 'any'),
                search=params['search_distance'], max_width=params['max_gate_width'],
                near='home', radius=params['home_radius'], **gate_options(params)),
    ])
    way = search
    if params.get('exit_source'):
        way = Selector('way out', memory=True, children=[
            Sequence('back the way in', memory=True, children=[
                PlanChainBack('plan the way out', 'exit_route', params['exit_source'],
                              params['corner_side'], params['gate_exit']),
                Navigate('out through the entry gates', 'exit_route', retries=2,
                         why='out through the entry gates'),
            ]),
            search,
        ])
    return Sequence('return home', memory=True, children=[way, Report('report home',
                                                                      'returned_home')])
