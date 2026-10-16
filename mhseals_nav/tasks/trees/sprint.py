"""Task 3, emergency sprint: through the gate, drive on until the beacon shows, circle it in
the direction its colour sets, and return through the gate."""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence

from mhseals_nav.tasks.behaviours import (
    Navigate, PlanCircle, PlanForward, PlanGates, PlanReturn, Report, WaitFor,
)


def build(params):
    red, green = params['red'], params['green']
    beacons = tuple(params['beacon_directions'])
    # Only the sighting ends the search; if the drive ends first, the boat waits there.
    sighting = WaitFor('beacon visible', beacons, any_of=True)
    seek = Parallel('seek beacon', policy=ParallelPolicy.SuccessOnSelected([sighting]), children=[
        Sequence('drive on', memory=True, children=[
            PlanForward('plan straight', 'straight', params['search_distance']),
            Navigate('drive straight', 'straight')]),
        sighting,
    ])
    return Sequence('sprint', memory=True, children=[
        WaitFor('gate visible', (red, green)),
        PlanGates('plan gate', 'gate', red, green, gates=1, max_width=params['max_gate_width']),
        Navigate('through gate', 'gate'),
        Report('report start', 'sprint_started'),  # the event's time starts the sprint clock
        seek,
        PlanCircle('plan circle', 'circle', beacons, params['circle_radius'],
                   clockwise=params['beacon_directions']),
        Report('report beacon', 'beacon_found', include=('circle_target',)),
        Navigate('circle beacon', 'circle'),
        PlanReturn('plan return', 'return', 'gate'),
        Navigate('back through gate', 'return'),
        Report('report finish', 'sprint_finished'),
    ])
