"""The whole run: every task in order, each under its own time limit.

The ``agenda`` tasks (a run of them in ``tasks``) go in the order the course-layout
belief plans (tasks/layout.py): the most points in the time left, the least time with
the way home, re-planned as each one ends; the rest keep their place.

The navigation channel must succeed (it is mandatory and first). A later task that
fails is tried again up to its ``attempts`` while its time limit lasts; then it is
reported as skipped and the run moves on, ending with the return home, which is
never skipped: a failed exit fails the run. A mandatory task that fails ends the
course, not the run: the boat still goes home, and the run fails. Only a human
taking control (the run is over) or a thruster no-go (the exit's own gate) stops it.
"""

import math

from mhseals_nav.tasks import budget
from mhseals_nav.tasks.search import course_side
from mhseals_nav.tasks.behaviours import Deadline, Leaf, Navigate, PlanLook, Report
from mhseals_nav.tasks.conditions import Gate, Watch
from py_trees.common import ParallelPolicy, Status
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import Decorator, FailureIsSuccess, Repeat


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

    def __init__(self, name, task, tasks, timeout, reserve, capabilities, limit_key,
                 planned=False):
        super().__init__(name, reads=('mission_started', 'plan'), writes=(limit_key,))
        self.task, self.tasks, self.timeout, self.reserve = task, tasks, timeout, reserve
        self.planned = planned  # an agenda task: the layout's plan decides
        self.capabilities, self.limit_key = capabilities, limit_key

    def update(self):
        started = self.board.get('mission_started') if self.board.exists(
            'mission_started') else None
        elapsed = self.board.now - started if started is not None else 0.0  # 0.0: a start
        left = self.timeout - elapsed - self.reserve
        here = [t['name'] for t in self.tasks].index(self.task)
        ok, why = budget.fits(self.task, left, self.tasks[here:], self.capabilities)
        plan = self.board.get('plan') if self.board.exists('plan') else None
        if self.planned and plan is not None and why.startswith(('no time', f'{self.task} fits')):
            # An agenda task: the layout's plan, from where it is, with the exit's reserve
            # from where the task ends (never more than the fixed one).
            ok = self.task in plan
            left = self.timeout - elapsed - plan.get(self.task, self.reserve)
            why = (f'{self.task} fits the plan: {left:.0f} s left' if ok
                   else f'no time for {self.task}: {left:.0f} s left')
        self.board.set(self.limit_key, max(left, 0.0), overwrite=True)
        if not ok:
            self.feedback_message = why
            self.decide('skip', why)
            return Status.FAILURE
        return Status.SUCCESS


class PlanNext(Leaf):
    """Pick the next of the ``agenda`` tasks by the layout's plan; write it to ``next_task``.

    The plan (tasks/layout.py) is redone from the boat's pose with everything seen so
    far: the order of the tasks left that fits the most points into the time left.
    It is written to ``plan`` ({name: the exit's reserve after it}) for the budget; a
    task out of it is still picked last, so its budget skips it with the reason.
    """

    def __init__(self, name, agenda, rows, timeout, reserve, side_cues, exit_via=None):
        super().__init__(name, reads=('mission_started', 'layout', 'next_task',
                                      *([exit_via] if exit_via else [])),
                         writes=('next_task', 'plan', 'agenda_done'))
        self.agenda, self.rows, self.timeout, self.reserve = agenda, rows, timeout, reserve
        self.side_cues, self.exit_via = side_cues, exit_via

    def update(self):
        done = self.board.get('agenda_done') if self.board.exists('agenda_done') else []
        left = [name for name in self.agenda if name not in (done or [])]
        layout, pose = self.board.get('layout'), self.board.pose
        if layout is None or pose is None or not left:
            return Status.FAILURE
        started = self.board.get('mission_started')
        time_left = self.timeout - (self.board.now - started)
        side = course_side(self.board.tracks, self.side_cues)
        guesses = {name: layout.guess(name, self.board.coverage, self.board.tracks, side)
                   for name in left} if self.board.coverage is not None else {}
        # The way out: back through the cleared entry gates, last first, then home.
        via = [*reversed(self.board.get(self.exit_via) or []), self.board.home or pose]
        plan = layout.plan(left, pose, via, time_left, guesses,
                           {name: self.rows[name] for name in left}, self.reserve)
        order = [name for name in plan] + [n for n in left if n not in plan]
        self.board.set('plan', {name: reserve for name, (_, reserve) in plan.items()},
                       overwrite=True)
        self.board.set('next_task', order[0], overwrite=True)
        self.board.set('agenda_done', [*(done or []), order[0]], overwrite=True)
        self.decide('plan', 'next: ' + ', '.join(order)
                    + (f' (none fits {time_left:.0f} s)' if not plan else ''))
        return Status.SUCCESS


class IsNext(Leaf):
    """SUCCESS when ``task`` is the agenda's ``next_task``."""

    def __init__(self, name, task):
        super().__init__(name, reads=('next_task',))
        self.task = task

    def update(self):
        return Status.SUCCESS if self.board.get('next_task') == self.task else Status.FAILURE


class SideKnown(Leaf):
    """SUCCESS once a side cue (``cues``: labels only side S carries) fixes the course side."""

    def __init__(self, name, cues):
        super().__init__(name)
        self.cues = tuple(cues)

    def update(self):
        known = course_side(self.board.tracks, self.cues) is not None
        return Status.SUCCESS if known else Status.FAILURE


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
                        search={**(params.get('search') or {}), 'task': name,
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
                        params['exit_reserve'], capabilities, limit,
                        name in (params.get('agenda') or ()))] if limit else []
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
    agenda = [n for n in params.get('agenda') or () if n in names]
    if agenda:
        at = [i for i, n in enumerate(names) if n in agenda]
        assert at == list(range(at[0], at[0] + len(at))), 'the agenda is a run of tasks'
        values = {r['name']: dict(r, value=budget.value(r, capabilities)) for r in rows}
        side_cues = (params.get('search') or {}).get('side_cues') or ()
        # One pass per agenda task: plan, then run the one picked (each runs once).
        turn = Sequence('agenda turn', memory=True, children=[
            PlanNext('agenda plan', agenda, values, params['timeout'], params['exit_reserve'],
                     side_cues, (params.get(EXIT) or {}).get('exit_source')),
            Selector('agenda task', memory=True, children=[
                Sequence(f'agenda {names[i]}', memory=True, children=[
                    IsNext(f'{names[i]} is next', names[i]), stages[i + 1]]) for i in at])])
        stages[at[0] + 1:at[-1] + 2] = [Repeat('agenda', turn, len(agenda))]
    look = params.get('start_look_deg')
    if look:
        stages.insert(1, start_look(math.radians(look),
                                    (params.get('search') or {}).get('side_cues') or ()))
    if names[-1] != EXIT:
        return Sequence('mission', memory=True, children=stages)
    course = CarryOn('course done', Sequence('course', memory=True, children=stages[:-1]))
    return Sequence('mission', memory=True, children=[
        course, stages[-1], Verdict('course verdict', course)])


def start_look(angle, side_cues=()):
    """Turn in place to each side and back before the first task: the cheapest survey.

    The course rules put the marina beside the start on side S (port or starboard), out
    of a forward camera's view; a turn either way finds it and fixes the side, which
    also tells where the speed challenge lies. A failed turn costs nothing but time.
    No turn back: the first task's goal turns the boat while it moves on.
    """
    look = Sequence('start look', memory=True, children=[
        Gate('start look go', False),
        Report('start look why', 'decision', kind='search',
               reason='look around the start: the marina lies to one side'),
        PlanLook('plan start look', 'start_look', angle),
        Navigate('look to one side', 'start_look', why='look around the start', phase='search'),
        # The side found at the first turn: no need to turn to the other.
        Selector('the other side', memory=True, children=[
            SideKnown('side found', side_cues),
            Navigate('look to the other', 'start_look_rest', why='look around the start',
                     phase='search')]),
    ])
    return FailureIsSuccess(name='start look done', child=look)
