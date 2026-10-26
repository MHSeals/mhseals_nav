"""The per-tick mission logic, free of ROS: the run_task node and the fast sim both drive it.

``MissionHost(task, params, navigator, sink, clock)`` holds one task tree (or the whole
mission) and everything a tick does around it: track memory and expiry, home and search
coverage, recognition, the stage table, the task_events/1 payloads and the end rules.
The caller supplies the world each tick (``tick(pose, tracks, fov)``) and the outside:

  navigator  sends routes: ``send(route, precise, quiet, why, reverse)`` -> goal with
             ``done``/``status``/``cancel()``; ``ready()``, ``keep_still()``, ``close()``;
             ``last`` (the latest goal, for the status) and ``report`` (set here)
  sink       ``sink(payload)``: every task_events/1 dict, as /task/events carries it
  clock      ``clock()`` -> seconds

Optional: ``actuators`` {capability: actuator} (tasks/actuators.py; only those the
``capabilities`` YAML turns on are used), ``status_sink(dict)`` gets mission_status/1
(on change, at most 5 Hz, and at 1 Hz), ``hypotheses_sink(dict)`` task_hypotheses/1 at
1 Hz, and ``on_tick(now)`` runs just before the tree ticks (the node feeds the
thruster-health check from it).
"""

import math
from pathlib import Path

from mhseals_nav.navigation_config import load_strict
from mhseals_nav.tasks import load_config, tracks as track_memory
from mhseals_nav.tasks.actuators import fitted
from mhseals_nav.tasks.conditions import Conditions
from mhseals_nav.tasks.recognition import Recogniser
from mhseals_nav.tasks.search import Coverage, load_config as load_search
from mhseals_nav.tasks.trees import KINDS, TREES
import py_trees

EVENTS_SCHEMA = 'mhseals.task_events/1'  # every /task/events message (test/fixtures)
OUTCOMES = ('SUCCESS', 'FAILURE', 'TIMEOUT', 'PREEMPTED', 'REFUSED')


def config_path(name):
    """Return a task YAML's path in the installed share, else in this source tree."""
    path = None
    try:
        from ament_index_python.packages import (get_package_share_directory,
                                                 PackageNotFoundError)
    except ImportError:  # no ROS on this host: the source tree only
        pass
    else:
        try:
            path = Path(get_package_share_directory('mhseals_nav')) / 'config/tasks' / name
        except PackageNotFoundError:
            pass
    if path is None or not path.exists():
        path = Path(__file__).resolve().parents[2] / 'config/tasks' / name
    return str(path)


def load_params(task, path=None):
    """Load a task's YAML; for the mission, each task's own YAML under its name."""
    path = path or config_path(f'{task}.yaml')
    params = load_strict(Path(path).read_text(encoding='utf-8'), path) or {}
    if task == 'mission':
        for name in params['tasks']:
            params[name] = {**load_params(name), **params.get(name, {})}
    return params


class MissionHost:
    STATUS_PERIOD, STATUS_MIN_GAP, RECENT = 1.0, 0.2, 5  # s, s, decisions kept in the status

    def __init__(self, task, params, navigator, sink, clock, status_sink=None,
                 hypotheses_sink=None, on_tick=None, actuators=None):
        self.task, self.navigator, self.sink, self.clock = task, navigator, sink, clock
        self.status_sink, self.hypotheses_sink, self.on_tick = (
            status_sink, hypotheses_sink, on_tick)
        self.config = load_config('runner', params.get('runner'))
        self.timeout = float(params.get('timeout', 600.0))
        self.objects, self.tracks, self.memory, self.priors = [], [], [], []
        self.memory_horizon = float(params.get('memory', 5.0))
        # Fixed course markers (a task's ``landmarks``) outlive ``memory``: see tick().
        # A slip light's colours (a task's ``indicators``) are states of one object.
        stages = params.get('tasks', [task] if task != 'mission' else [])
        configs = [params] + [params.get(n) or {} for n in stages]
        self.landmarks = dict.fromkeys(
            [label for p in configs for label in p.get('landmarks', ())],
            float(params.get('landmark_memory', 900.0)))
        self.exclusive = {label for p in configs for label in p.get('indicators') or ()}
        search = load_search(params.get('search'))
        self.coverage_config = search
        self.fov = (math.radians(float(search['hfov_deg'])), float(search['range_m']))
        self.conditions = Conditions(params.get('conditions') or {})
        self.recogniser = Recogniser(load_config('recognition'))
        self.status_at, self.status_last, self.hypotheses_at = None, None, None
        self.stages = {name: {'state': 'pending', 'attempts': 0, 'failures': 0,
                              'elapsed': 0.0, 'since': None} for name in stages}
        self.stage, self.recent = None, []
        self.owner = None  # who took control, named in the give_up decision
        self.preempting = self.ticking = False
        self.board = py_trees.blackboard.Client(name='runner')
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage', 'hypotheses',
                    'conditions'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.board.home = self.board.coverage = None
        self.board.conditions = self.conditions
        self.board.fov = self.fov
        self.tree = py_trees.trees.BehaviourTree(TREES[task](params))
        navigator.report = self.report
        # Every capability is off unless the YAML turns it on and an actuator is given.
        self.actuators = fitted(params.get('capabilities'), actuators)
        self.tree.setup(timeout=15.0, navigator=navigator, report=self.report,
                        preempt=self.preempt, runner=self.config, actuators=self.actuators)
        self.started = None
        self.outcome = None

    # World memory -------------------------------------------------------------------

    def remember(self, seen, priors=True):
        """Remember one tracker message's tracks, seen now.

        Its course-map priors (``prior`` true) replace the remembered ones, unless
        ``priors`` is false (a source that carries none, such as /objects/map).
        """
        if priors:
            self.priors = [track for track in seen if track.prior]  # republished while held
        self.memory = track_memory.remember(self.memory, seen, self.clock(),
                                            self.memory_horizon, horizons=self.landmarks,
                                            exclusive=self.exclusive)
        self._world()

    def _world(self):
        """Remembered live tracks, plus the latest course-map priors (kept as tracks expire)."""
        remembered = [track for track, _ in self.memory]
        self.tracks = remembered + self.priors
        self.objects = track_memory.objects(remembered)

    # Events -------------------------------------------------------------------------

    def decide(self, kind, reason, **details):
        self.report('decision', kind=kind, reason=reason[:60], **details)

    def report(self, event, **details):
        now = self.clock()
        if event == 'decision' and self.stage and 'stage' not in details:
            details['stage'] = self.stage
        payload = {'schema': EVENTS_SCHEMA, 'task': self.task, 'event': event, 'time': now,
                   'elapsed': now - self.started if self.started is not None else 0.0,
                   **details}
        self.sink(payload)
        self.follow(event, details, payload)

    def follow(self, event, details, payload):
        """Keep the mission's stage table from the events the trees report."""
        now, stage = payload['time'], self.stages.get(details.get('stage'))
        if event == 'decision':
            self.recent = (self.recent + [payload])[-self.RECENT:]
        elif event == 'stage_started' and stage is not None:
            self.stage = details['stage']
            stage.update(state='running', since=now)
            stage['attempts'] += 1
        elif event in ('stage_completed', 'stage_skipped', 'stage_failed') and stage:
            kind = KINDS.get(details['stage'])
            if kind and event != 'stage_failed':
                self.recogniser.mark(kind, 'done' if event == 'stage_completed' else 'failed')
            stage['state'] = {'stage_completed': 'done', 'stage_skipped': 'skipped',
                              'stage_failed': 'running'}[event]
            if event == 'stage_failed':
                stage['failures'] += 1
            if stage['since'] is not None:
                stage['elapsed'] = round(now - stage['since'], 1)
            if event != 'stage_failed':
                self.stage = None
        else:
            return
        self.status_at = None  # publish on change

    # The tick -----------------------------------------------------------------------

    def tick(self, pose, tracks=None, fov=None):
        """Run one tick; return the outcome once the run has ended, else None.

        ``tracks`` (when given; the node passes None and calls ``remember`` from its
        callbacks instead) is what one object-tracker message carries, as
        ``tasks.tracks.Track`` (label, x, y in the map frame, confidence, sigma, static,
        prior, id): every confirmed track the tracker still holds, out of view ones
        included, not only this frame's detections. The tracker (config/tracker.yaml)
        holds a track ``timeout`` s after its last sighting by its motion class: STATIC
        900 s, UNKNOWN 10 s, MOVING 3 s; and drops a STATIC one sooner when it stands in
        the camera's view and range unseen for ``miss_in_fov_s`` (5 s). A buoy turns
        STATIC; a class of no group (banners, light beacons) seen from beyond ~9 m stays
        UNKNOWN; a boat is MOVING. ``confidence`` is the class score times
        exp(-age / tau) (tau 300 s STATIC, 5 s UNKNOWN, 2 s MOVING); the last-seen
        stamp itself is not passed. ``id`` is the tracker's, stable while it holds the
        track. Course-map priors come with ``prior`` true. Every track given is
        remembered as reported now and kept ``memory`` s (5) after its last report,
        a ``landmarks`` label ``landmark_memory`` s (900): the task logic must not
        need any other track longer than the tracker holds it. ``fov`` (hfov rad,
        range m) replaces the camera's field of view. The run starts at the first tick
        with a pose and a ready navigator.
        """
        if self.outcome is not None:
            return self.outcome
        if fov is not None:
            self.fov = fov
        if tracks is not None:
            self.remember(tracks)
        now = self.clock()
        if self.started is None:
            # A goal sent before discovery has matched Nav2's action servers is lost,
            # and its future never completes: the tree would wait on it for good.
            # Planning also needs the initial map pose (and a valid home).
            if pose is None or not self.navigator.ready():
                return None
            self.started = now
            self.report('task_started')
        # Forget tracks the tracker stopped sending: it may have died.
        if self.memory:
            before = len(self.memory)
            self.memory = track_memory.remember(self.memory, [], now, self.memory_horizon,
                                                horizons=self.landmarks,
                                                exclusive=self.exclusive)
            if len(self.memory) != before:
                self._world()
        self.board.objects, self.board.tracks = self.objects, self.tracks
        self.board.pose, self.board.now, self.board.fov = pose, now, self.fov
        if self.board.home is None:
            self.board.home = self.board.pose  # where the run started, for the return
            self.report('home', x=round(pose[0], 2), y=round(pose[1], 2))
            c = self.coverage_config
            self.board.coverage = Coverage(c['area'], c['cell'], pose, c['radius'],
                                           c['forget_s'])
        if self.board.coverage is not None:
            self.board.coverage.observe(pose, self.fov, now)
        if self.on_tick is not None:
            self.on_tick(now)
        self.recognise(now)
        self.ticking = True
        try:
            self.tree.tick()
        finally:
            self.ticking = False
        status = self.tree.root.status.name
        if self.preempting:
            self.finish('PREEMPTED')
        elif status in ('SUCCESS', 'FAILURE'):
            self.finish(status)
        elif now - self.started > self.timeout:
            self.finish('TIMEOUT')
        else:
            self.navigator.keep_still()  # after the tree, so a goal it just sent counts
            self.publish_status(status)
        return self.outcome

    def recognise(self, now):
        if self.recogniser is None:
            return
        self.board.hypotheses = self.recogniser.update(self.tracks, self.board.home, now)
        if self.hypotheses_at is None or now - self.hypotheses_at >= self.STATUS_PERIOD:
            self.hypotheses_at = now
            if self.hypotheses_sink is not None:
                self.hypotheses_sink(self.recogniser.message(now))

    def status(self, status):
        """Build the mission_status/1 dict for the tree's ``status`` (its name)."""
        now = self.clock()
        tip = self.tree.root.tip()
        stages = {name: {k: v for k, v in stage.items() if k != 'since'}
                  for name, stage in self.stages.items()}
        for name, stage in self.stages.items():
            if stage['state'] == 'running' and stage['since'] is not None:
                stages[name]['elapsed'] = round(now - stage['since'], 1)
        return {
            'schema': 'mhseals.mission_status/1', 'task': self.task,
            'status': self.outcome or status, 'elapsed': round(now - (self.started or now), 1),
            'tip': tip.name if tip else None,
            'stage': self.stage or (self.task if self.task != 'mission' else None),
            'phase': getattr(tip, 'phase', None) if self.outcome is None else 'done',
            'behaviour': tip.name if tip else None,
            'goal': getattr(self.navigator, 'last', None),
            'stages': stages,
            'remaining': [n for n, st in self.stages.items() if st['state'] == 'pending'],
            'recent': self.recent}

    def publish_status(self, status, force=False):
        now = self.clock()
        if not force and self.status_at is not None and (
                now - self.status_at < self.STATUS_PERIOD):
            return
        if not force and self.status_last is not None and now - self.status_last < (
                self.STATUS_MIN_GAP):
            return
        self.status_at = self.status_last = now
        if self.status_sink is not None:
            self.status_sink(self.status(status))

    # Ends ---------------------------------------------------------------------------

    def refuse(self):
        """Do not start while a human has control, nor later by itself."""
        self.decide('give_up', f'{self.owner} has control: not starting')
        self.report('task_refused', owner=self.owner)
        self.finish('REFUSED')

    def preempt(self, why=None):
        """Yield to the human at once: cancel every goal and end; never resume.

        From inside a tick (a leaf saw its goal cancelled by someone else) the tree is
        stopped once the tick returns; no goal goes out meanwhile.
        """
        if self.outcome is not None or self.preempting:
            return
        self.preempting = True
        self.navigator.close()
        why = why or f'{self.owner or "human"} override'
        self.decide('give_up', f'{why}: stopped, no resume')
        self.report('task_suspended', owner=self.owner)
        if not self.ticking:
            self.finish('PREEMPTED')

    def finish(self, outcome):
        self.outcome = outcome
        self.navigator.close()  # every goal sent is cancelled; none goes out after
        self.tree.root.stop(py_trees.common.Status.INVALID)
        self.report('task_finished', outcome=outcome)
        if self.started is not None:
            self.publish_status(outcome, force=True)
