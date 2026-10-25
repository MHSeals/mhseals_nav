"""Task 4, speed challenge: out through the gate, around the blue mark, and back.

The black buoys (oil spills) past the gate are counted and reported before
leaving. There is no light panel yet, so the run starts at once; the time from the
gate to the gate is what scores, so once the mark is known the lap and the return
through the gate are one route: the boat never stops between them.
"""

from mhseals_nav.tasks.behaviours import (
    CountObjects, GateRun, Navigate, PlanCircle, PlanConcat, PlanGatePass, PlanView, Report,
    ReportCount, Search,
)
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence


def build(params):
    from mhseals_nav.tasks.trees import gate_options, look_for
    red, green, mark = params['red'], params['green'], params['mark']
    # Without the day's layout, the blue mark (only this task has one) leads the boat to
    # the task; red/green gates look alike, so other tasks' gates are excluded.
    near, radius, find = 'speed_challenge_approach', params['gate_radius'], []
    if not params.get('approach'):
        near, radius = 'speed_mark', params['mark_radius']
        find = [look_for('find the mark first', (mark,), params),
                PlanView('plan a view of the mark', 'speed_view', mark),
                Navigate('to the mark', 'speed_view', accept=3.0, why='to the speed mark',
                         phase='approach'),
                PlanView('where the mark is', 'speed_mark', mark, standoff=0.0)]
    run = Sequence('speed run', memory=True, children=find + [
        look_for('find the gate', (red, green), params),
        GateRun('through the gate', 'speed_gate', red, green, gates=1,
                max_width=params['max_gate_width'], near=near, radius=radius,
                exclude=('path', 'channel'), exclude_radius=params['exclude_radius'],
                toward=mark, **gate_options(params)),
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
