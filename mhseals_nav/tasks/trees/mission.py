"""The whole run: every task in order, each under its own time limit.

The navigation channel must succeed (it is mandatory and first). A later task that
fails is tried again up to its ``attempts`` while its time limit lasts; then it is
reported as skipped and the run moves on, ending with the return home, which is
never skipped: a failed exit fails the run. A mandatory task that fails ends the
course, not the run: the boat still goes home, and the run fails. Only a human
taking control (the run is over) or a thruster no-go (the exit's own gate) stops it.
"""

import math

from mhseals_nav.tasks import budget
from mhseals_nav.tasks.behaviours import Deadline, Leaf, Navigate, PlanLook, Report
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


class Budget(Leaf):
    """Before an optional task: FAILURE (skip it) when it does not fit the time left.

    The time left is the run's ``timeout`` from the first tick, less the exit
    ``reserve``. It also writes the task's own limit (``limit_key``): no task may eat
    the time the exit needs.
    """

    def __init__(self, name, task, tasks, timeout, reserve, capabilities, limit_key):
        super().__init__(name, reads=('mission_started',), writes=(limit_key,))
        self.task, self.tasks, self.timeout, self.reserve = task, tasks, timeout, reserve
        self.capabilities, self.limit_key = capabilities, limit_key

    def update(self):
        started = self.board.get('mission_started') if self.board.exists(
            'mission_started') else None
        left = self.timeout - (self.board.now - (started or self.board.now)) - self.reserve
        here = [t['name'] for t in self.tasks].index(self.task)
        ok, why = budget.fits(self.task, left, self.tasks[here:], self.capabilities)
        self.board.set(self.limit_key, max(left, 0.0), overwrite=True)
        if not ok:
            self.feedback_message = why
            self.decide('skip', why)
            return Status.FAILURE
        return Status.SUCCESS


class StartClock(Leaf):
    """Write the run's start time once: the budget counts from here."""

    def __init__(self, name):
        super().__init__(name, reads=('mission_started',), writes=('mission_started',))

    def update(self):
        if not (self.board.exists('mission_started') and self.board.mission_started is not None):
            self.board.set('mission_started', self.board.now, overwrite=True)
        return Status.SUCCESS


class CarryOn(Decorator):
    """Turn the course's failure into success, so the exit still runs; remember it.

    Not once the run is closed (preempted): then nothing more may go out.
    """

    def setup(self, **kwargs):
        self.report, self.navigator = kwargs.get('report'), kwargs.get('navigator')

    def initialise(self):
        self.failed = False

    def update(self):
        if self.decorated.status != Status.FAILURE:
            return self.decorated.status
        if getattr(self.navigator, 'closed', False):
            return Status.FAILURE
        self.failed = True
        self.report('decision', kind='give_up', reason=f'course failed: on to {EXIT}')
        return Status.SUCCESS


class Verdict(Leaf):
    """After the exit: FAILURE when the course before it failed (``course`` CarryOn)."""

    def __init__(self, name, course):
        super().__init__(name)
        self.course = course

    def update(self):
        return Status.FAILURE if self.course.failed else Status.SUCCESS


EXIT = 'return_home'


def mandatory(params):
    """Return the tasks never skipped: the ``mandatory`` list, and always the exit."""
    return {*(params.get('mandatory') or ()), EXIT}


def plan(params):
    """Return the optional tasks' budget rows (name, points, expected_s, needs) in order.

    The first task and the ``mandatory`` ones (the exit) are not chosen: they always run.
    """
    return [{'name': name, 'points': params[name].get('points', 1.0),
             'expected_s': params[name].get('expected_s', 0.0),
             'needs': params[name].get('needs') or []}
            for name in params['tasks'][1:] if name not in mandatory(params)]


def build(params):
    from mhseals_nav.tasks.trees import TREES
    stages, names = [StartClock('start the clock')], params['tasks']
    rows, capabilities = plan(params), params.get('capabilities') or {}
    for index, name in enumerate(names):
        # The mission's search settings (its course area) apply to every task.
        settings = dict(params[name], capabilities=capabilities,
                        search={**(params.get('search') or {}),
                                **(params[name].get('search') or {})})
        following = names[index + 1] if index + 1 < len(names) else None
        precise = settings.get('precise', False)
        attempts = Attempts(f'{name} attempts', TREES[name](settings), name,
                            settings.get('attempts', 1))
        # The boat's state can stop a task (tasks/conditions.py): never start one.
        watched = Parallel(f'{name} watched', policy=ParallelPolicy.SuccessOnSelected(
            [attempts]), children=[attempts, Watch(f'{name} fit', precise)])
        # Optional tasks give up at their own limit or when the exit's reserve is due.
        optional = index > 0 and name not in mandatory(params)
        limit = f'{name}_budget_s' if optional and 'exit_reserve' in params else None
        task = Deadline(f'{name} time', watched, settings['time_limit'], limit_key=limit)
        admit = [Budget(f'{name} budget', name, rows, params['timeout'],
                        params['exit_reserve'], capabilities, limit)] if limit else []
        attempt = Sequence(f'{name} attempt', memory=True, children=[
            *admit,
            Report(f'{name} start', 'stage_started', stage=name),
            Report(f'{name} decision', 'decision', stage=name, kind='start',
                   reason=f'starting {name}'),
            Gate(f'{name} go', precise),
            task,
            Report(f'{name} done', 'stage_completed', stage=name),
        ])
        if not optional:
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
        stages.insert(1, start_look(math.radians(look)))
    if names[-1] != EXIT:
        return Sequence('mission', memory=True, children=stages)
    course = CarryOn('course done', Sequence('course', memory=True, children=stages[:-1]))
    return Sequence('mission', memory=True, children=[
        course, stages[-1], Verdict('course verdict', course)])


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
