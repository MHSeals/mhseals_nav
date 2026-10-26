"""Task 4, supply drop: water on the triangle hulls, balls into the plus hulls.

Neither actuator is built yet: each kind ``needs`` a capability (tasks/actuators.py)
and is tried only when it is on; the mission's budget counts no points for a kind
that is off. Each kind: find a hull, view it from ``standoff``, deliver. The task
succeeds once anything was delivered.
"""

from mhseals_nav.tasks.behaviours import Deliver, Leaf, Navigate, PlanView
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import FailureIsSuccess


class Delivered(Leaf):
    """Condition: SUCCESS once ``key`` lists at least one delivery."""

    def __init__(self, name, key='delivered'):
        super().__init__(name, reads=(key,))
        self.key = key

    def update(self):
        done = self.board.get(self.key) if self.board.exists(self.key) else None
        return Status.SUCCESS if done else Status.FAILURE


def build(params):
    from mhseals_nav.tasks.trees import look_for
    capabilities = params.get('capabilities') or {}
    kinds = []
    for kind, spec in params['kinds'].items():
        if not capabilities.get(spec['needs'], False):
            continue  # not fitted: nothing to try
        labels = tuple(spec['labels'])
        for n in range(spec.get('count', 1)):
            kinds.append(FailureIsSuccess(name=f'{kind} {n + 1} or not', child=Sequence(
                f'{kind} {n + 1}', memory=True, children=[
                    look_for(f'find a {kind} hull', labels, params, need='any'),
                    PlanView(f'plan {kind} view', 'supply_view', labels[0],
                             standoff=spec['standoff']),
                    Navigate(f'to the {kind} hull', 'supply_view', accept=1.0,
                             why=f'to a {kind} hull', phase='approach'),
                    Deliver(f'deliver {kind}', spec['needs'], labels)])))
    return Sequence('supply drop', memory=True, children=[*kinds, Delivered('anything in')])
