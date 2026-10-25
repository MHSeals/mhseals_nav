"""Task 4, speed challenge: out through the gate, around the blue mark, and back.

The black buoys (oil spills) past the gate are counted and reported before
leaving. There is no light panel yet, so the run starts at once; the time from the
gate to the gate is what scores, so once the mark is known the lap and the return
through the gate are one route: the boat never stops between them.
"""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence

from mhseals_nav.tasks.behaviours import (
    CountObjects, GateRun, Navigate, PlanCircle, PlanConcat, PlanGatePass, Report, ReportCount,
    Search,
)


def build(params):
    from mhseals_nav.tasks.trees import look_for
    red, green, mark = params['red'], params['green'], params['mark']
    run = Sequence('speed run', memory=True, children=[
        look_for('find the gate', (red, green), params),
        # Its gate is the one by its approach: other tasks' red/green gates look alike.
        GateRun('through the gate', 'speed_gate', red, green, gates=1,
                max_width=params['max_gate_width'], near='speed_challenge_approach',
                radius=params['gate_radius']),
        Report('report start', 'speed_started'),
        # The mark lies straight on past the gate: drive on, and search if it is not there.
        Selector('find the mark', memory=True, children=[
            Search('drive on to the mark', (mark,), params['search_distance']),
            look_for('search for the mark', (mark,), params)]),
        PlanCircle('plan circle', 'circle', (mark,), params['circle_radius'],
                   clockwise=params['circle_clockwise'], points=params['circle_points']),
        PlanGatePass('plan return', 'return', 'speed_gate', back=True),
        PlanConcat('plan lap', 'lap', ('circle', 'circle_rest', 'return')),
        Navigate('round the mark and back through the gate', 'lap', retries=2,
                 why='round the mark, back through the gate'),
        ReportCount('report count', 'spills', 'spill_count', spin=params['report'] == 'spin'),
        Report('report finish', 'speed_finished'),
    ])
    count = CountObjects('count spills', 'spills', params['count'], beyond='speed_gate')
    return Parallel('speed challenge', policy=ParallelPolicy.SuccessOnSelected([run]),
                    children=[run, count])
