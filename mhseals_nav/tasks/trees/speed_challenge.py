"""Task 4, speed challenge: out through the gate, around the blue mark, and back.

The black buoys (oil spills) past the gate are counted and reported before
leaving. There is no light panel yet, so the run starts at once; the time from the
gate to the gate is what scores.
"""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence

from mhseals_nav.tasks.behaviours import (
    CountObjects, GateRun, Navigate, PlanCircle, PlanGatePass, Report, ReportCount, Search,
    WaitFor,
)


def build(params):
    red, green, mark = params['red'], params['green'], params['mark']
    run = Sequence('speed run', memory=True, children=[
        WaitFor('gate visible', (red, green)),
        GateRun('through the gate', 'speed_gate', red, green, gates=1,
                max_width=params['max_gate_width']),
        Report('report start', 'speed_started'),
        Search('find the mark', (mark,), params['search_distance']),
        PlanCircle('plan circle', 'circle', (mark,), params['circle_radius'],
                   clockwise=params['circle_clockwise'], points=params['circle_points']),
        Navigate('circle the mark', 'circle'),
        ReportCount('report count', 'spills', 'spill_count', spin=params['report'] == 'spin'),
        PlanGatePass('plan return', 'return', 'speed_gate', back=True),
        Navigate('back through the gate', 'return'),
        Report('report finish', 'speed_finished'),
    ])
    count = CountObjects('count spills', 'spills', params['count'], beyond='speed_gate')
    return Parallel('speed challenge', policy=ParallelPolicy.SuccessOnSelected([run]),
                    children=[run, count])
