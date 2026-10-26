"""The whole run: every task in order, each under its own time limit.

The navigation channel must succeed (it is mandatory and first). A later task that
fails is tried again up to its ``attempts`` while its time limit lasts; then it is
reported as skipped and the run moves on, ending with the return home.
"""

import math

from mhseals_nav.tasks.behaviours import Deadline, Navigate, PlanLook, Report
from mhseals_nav.tasks.conditions import Gate, Watch
from py_trees.common import ParallelPolicy, Status
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import Decorator, FailureIsSuccess


class Attempts(Decorator):
    """Re-run a failed child up to ``attempts`` times in all, reporting each failure."""

    def __init__(self, name, child, stage, attempts):
        super().__init__(name=name, child=child)
        self.stage, self.attempts = stage, attempts

    def setup(self, **kwargs):
        self.report, self.navigator = kwargs.get('report'), kwargs.get('navigator')

    def initialise(self):
        self.failures = 0

    def update(self):
        if self.decorated.status != Status.FAILURE:
            return self.decorated.status
        if getattr(self.navigator, 'closed', False):  # preempted: the run is over
            return Status.FAILURE
        self.failures += 1
        last = self.failures >= self.attempts
        why = self.decorated.tip().feedback_message if self.decorated.tip() else ''
        self.report('stage_failed', stage=self.stage, attempt=self.failures, why=why)
        self.report('decision', stage=self.stage, kind='give_up' if last else 'retry',
                    reason=(f'{self.stage} failed {self.failures}x: giving up' if last else
                            f'{self.stage} failed: attempt {self.failures + 1}')[:60])
        if last:
            return Status.FAILURE
        self.decorated.stop(Status.INVALID)  # the next tick starts it afresh
        return Status.RUNNING


def build(params):
    from mhseals_nav.tasks.trees import TREES
    stages, names = [], params['tasks']
    for index, name in enumerate(names):
        # The mission's search settings (its course area) apply to every task.
        settings = dict(params[name], search={**(params.get('search') or {}),
                                              **(params[name].get('search') or {})})
        following = names[index + 1] if index + 1 < len(names) else None
        precise = settings.get('precise', False)
        attempts = Attempts(f'{name} attempts', TREES[name](settings), name,
                            settings.get('attempts', 1))
        # The boat's state can stop a task (tasks/conditions.py): never start one.
        watched = Parallel(f'{name} watched', policy=ParallelPolicy.SuccessOnSelected(
            [attempts]), children=[attempts, Watch(f'{name} fit', precise)])
        task = Deadline(f'{name} time', watched, settings['time_limit'])
        attempt = Sequence(f'{name} attempt', memory=True, children=[
            Report(f'{name} start', 'stage_started', stage=name),
            Report(f'{name} decision', 'decision', stage=name, kind='start',
                   reason=f'starting {name}'),
            Gate(f'{name} go', precise),
            task,
            Report(f'{name} done', 'stage_completed', stage=name),
        ])
        if index == 0:
            stages.append(attempt)  # mandatory
        else:
            stages.append(Selector(f'{name} or skip', memory=True, children=[
                attempt, Sequence(f'{name} skip', memory=True, children=[
                    Report(f'{name} skipped', 'stage_skipped', stage=name),
                    Report(f'{name} switch', 'decision', stage=name, kind='skip',
                           reason=f'skipping {name}'
                           + (f': on to {following}' if following else '')),
                ])]))
    look = params.get('start_look_deg')
    if look:
        stages.insert(0, start_look(math.radians(look)))
    return Sequence('mission', memory=True, children=stages)


def start_look(angle):
    """Turn in place to each side and back before the first task: the cheapest survey.

    The course rules put the marina beside the start on side S (port or starboard), out
    of a forward camera's view; a turn either way finds it and fixes the side, which
    also tells where the speed challenge lies. A failed turn costs nothing but time.
    """
    look = Sequence('start look', memory=True, children=[
        Gate('start look go', False),
        Report('start look why', 'decision', kind='search',
               reason='look around the start: the marina lies to one side'),
        PlanLook('plan start look', 'start_look', angle),
        Navigate('look to one side', 'start_look', why='look around the start', phase='search'),
        Navigate('look to the other', 'start_look_rest', why='look around the start',
                 phase='search'),
        Navigate('face the course', 'start_look_back', why='face the course', phase='search'),
    ])
    return FailureIsSuccess(name='start look done', child=look)
