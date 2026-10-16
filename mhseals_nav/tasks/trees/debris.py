"""Task 2, debris clearance: transit the channel, report debris and beacons, circle a
survivor beacon if there is one, then return through the channel."""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence

from mhseals_nav.tasks.behaviours import (
    Navigate, PlanCircle, PlanGates, PlanReturn, Report, ReportObjects, Seen, WaitFor,
)


def build(params):
    red, green = params['red'], params['green']
    survivor = Sequence('circle survivor', memory=True, children=[
        Seen('survivor beacon seen', params['survivor']),
        PlanCircle('plan circle', 'circle', params['survivor'], params['circle_radius'],
                   clockwise=params['circle_clockwise']),
        Navigate('circle beacon', 'circle'),
        Report('report circle', 'survivor_circled', include=('circle_target',)),
    ])
    mission = Sequence('debris mission', memory=True, children=[
        WaitFor('channel visible', (red, green)),
        PlanGates('plan channel', 'channel', red, green, gates=params['channel_gates'],
                  max_width=params['max_gate_width']),
        Navigate('transit channel', 'channel'),
        Report('report channel', 'channel_transited', include=('channel',)),
        Selector('survivor', memory=True, children=[
            survivor, Report('report none', 'no_survivor_found')]),
        PlanReturn('plan return', 'return', 'channel'),
        Navigate('return through channel', 'return'),
        Report('report return', 'returned'),
    ])
    sightings = Parallel('report sightings', policy=ParallelPolicy.SuccessOnAll(), children=[
        ReportObjects('report debris', 'debris', params['debris']),
        ReportObjects('report beacons', 'beacon', params['beacons']),
    ])
    return Parallel('debris', policy=ParallelPolicy.SuccessOnSelected([mission]),
                    children=[mission, sightings])
