"""Speed challenge: out through the gate, around the mark, and back.

On the 2026 course (``indicators`` set) the colour indicator between the gate
and the mark sets the way round: ``turn`` maps its colour to clockwise.

The black buoys (oil spills) past the gate are counted and reported before
leaving. There is no light panel yet, so the run starts at once; the time from the
gate to the gate is what scores, so once the mark is known the lap and the return
through the gate are one route: the boat never stops between them.
"""

from mhseals_nav.tasks.behaviours import (
    CountObjects, GateRun, Navigate, PlanCircle, PlanConcat, PlanGatePass, PlanView, Report,
    ReadColour, ReportCount, Search,
)
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence


def build(params):
    from mhseals_nav.tasks.trees import gate_options, look_for
    red, green, mark = params['red'], params['green'], params['mark']
    indicators = params.get('indicators') or {}
    # Without the day's layout, the blue mark (only this task has one) leads the boat to
    # the task; red/green gates look alike, so other tasks' gates are excluded.
    near, radius, find = 'speed_challenge_approach', params['gate_radius'], []
    if not params.get('approach'):
        near, radius = 'speed_mark', params['mark_radius']
        # A look-alike mark elsewhere (the harbor buoy) has no gate: ``pair`` needs one.
        pair = params.get('pair')
        find = [look_for('find the mark first', (mark,), params, pair=pair),
                PlanView('plan a view of the mark', 'speed_view', mark, pair=pair),
                Navigate('to the mark', 'speed_view', accept=3.0, why='to the speed mark',
                         phase='approach'),
                PlanView('where the mark is', 'speed_mark', mark, standoff=0.0, pair=pair)]
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
        # 2026: the indicator between the gate and the mark sets the way round; it is
        # read now, with the gate and the mark known, just before the lap is planned.
        *([ReadColour('read the indicator', 'speed_colour', indicators, 'speed_gate',
                      (mark,), params.get('indicator_reach', 8.0))] if indicators else []),
        PlanCircle('plan circle', 'circle', (mark,), params['circle_radius'],
                   clockwise=params['circle_clockwise'], points=params['circle_points'],
                   direction=('speed_colour', params['turn']) if indicators else None),
        PlanGatePass('plan return', 'return', 'speed_gate', back=True),
        PlanConcat('plan lap', 'lap', ('circle', 'circle_rest', 'return')),
        Navigate('round the mark and back through the gate', 'lap', retries=2,
                 why='round the mark, back through the gate'),
        ReportCount('report count', 'spills', 'spill_count', spin=params['report'] == 'spin'),
        Report('report finish', 'speed_finished'),
    ])
    watchers = [CountObjects('count spills', 'spills', params['count'], beyond='speed_gate')]
    return Parallel('speed challenge', policy=ParallelPolicy.SuccessOnSelected([run]),
                    children=[run, *watchers])
