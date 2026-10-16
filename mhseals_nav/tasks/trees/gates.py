"""Task 1, navigation channel: pass through the entry gate and then the exit gate.

When only the entry gate is visible, the boat passes it and drives on until the
exit gate appears, then passes that.
"""

from py_trees.common import ParallelPolicy, Status
from py_trees.composites import Parallel, Selector, Sequence

from mhseals_nav.tasks.behaviours import (
    Leaf, Navigate, PlanForward, PlanGates, Report, WaitFor,
)


class RouteHas(Leaf):
    """Condition: the route in ``key`` holds at least ``count`` waypoints."""

    def __init__(self, name, key, count):
        super().__init__(name, reads=(key,))
        self.key, self.count = key, count

    def update(self):
        return Status.SUCCESS if len(self.board.get(self.key)) >= self.count else Status.FAILURE


def build(params):
    red, green = params['red'], params['green']
    search = Sequence('search ahead', memory=True, children=[
        PlanForward('plan search', 'search', params['search_distance']),
        Navigate('drive ahead', 'search'),
    ])
    # Only the sighting ends the search; if the drive ends first, the boat waits there.
    sighting = WaitFor('exit gate visible', (red, green), count=2)
    exit_gate = Sequence('find exit gate', memory=True, children=[
        Parallel('drive until seen', policy=ParallelPolicy.SuccessOnSelected([sighting]),
                 children=[search, sighting]),
        PlanGates('plan exit gate', 'exit', red, green, gates=1, skip=1,
                  max_width=params['max_gate_width']),
        Navigate('through exit gate', 'exit'),
        Report('report exit', 'gate_cleared', gate='exit'),
    ])
    return Sequence('gates', memory=True, children=[
        WaitFor('gate visible', (red, green)),
        PlanGates('plan gates', 'gates', red, green, gates=2, max_width=params['max_gate_width']),
        Navigate('through gates', 'gates'),
        Report('report gates', 'gate_cleared', include=('gates',)),
        Selector('exit gate', memory=True, children=[
            RouteHas('both gates passed', 'gates', 2), exit_gate]),
    ])
