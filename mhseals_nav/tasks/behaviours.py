"""Reusable task leaves. Each reads and writes the blackboard; only Navigate talks to Nav2.

Blackboard keys filled by the runner every tick:
  objects   live tracked objects as (label, x, y) in the map frame
  tracks    every track as tasks.tracks.Track, priors and remembered ones included
  pose      the boat's (x, y, yaw) in the map frame, or None
  now       ROS time in seconds
  home      where the run started (x, y, yaw), for the return
  fov       the camera's (hfov rad, range m)
  coverage  tasks.search.Coverage: where the camera has looked
Leaves pass routes to each other through keys named in their arguments.
"""

import math

from mhseals_nav.tasks import geometry, load_config
import py_trees
from py_trees.common import Access, Status
from py_trees.decorators import Decorator


class Leaf(py_trees.behaviour.Behaviour):
    """A behaviour with a blackboard client for the shared world keys.

    ``phase`` (approach, search, execute, report) is what the mission status shows.
    """

    phase = 'execute'

    def __init__(self, name, reads=(), writes=()):
        super().__init__(name)
        self.board = self.attach_blackboard_client(name=name)
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage', *reads):
            self.board.register_key(key=key, access=Access.READ)
        for key in writes:
            self.board.register_key(key=key, access=Access.WRITE)

    def setup(self, **kwargs):
        self.navigator, self.report = kwargs.get('navigator'), kwargs.get('report')
        self.preempt = kwargs.get('preempt')  # the runner's: end the task, never resume

    def decide(self, kind, reason, **details):
        """Publish a decision: a short reason the operator reads in the TUI log."""
        if self.report is not None:
            self.report('decision', kind=kind, reason=reason[:60], **details)

    def labelled(self, *labels):
        return [(x, y) for label, x, y in self.board.objects if label in labels]


def action(route, precise):
    """Which Nav2 action a route goes to (see interface.Navigator.send)."""
    return 'to_pose' if len(route) == 1 or precise else 'through_poses', precise


class Driver(Leaf):
    """A leaf that drives: one Nav2 goal at a time, handed over cleanly.

    ``drive(route)`` replaces whatever this leaf is driving. Nav2 runs one navigation
    at a time and refuses a new one while another winds down, so a replacement waits
    for the cancelled goal to finish, and a refused goal is sent again. Call
    ``driving()`` every tick: RUNNING while a goal is on, else the last goal's end
    (SUCCEEDED, ABORTED, ...), or None when nothing was sent. Any end of the leaf
    cancels its goal. A goal cancelled by anyone but the task preempts the run.
    """

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # goal_retries (sends of one route) and refusal_s: config/tasks/runner.yaml
        self.limits = kwargs.get('runner') or load_config('runner')

    def initialise(self):
        self.goal = self.old = self.queued = self.outcome = self.anchor = None
        self.superseded = []

    def drive(self, route, precise=False, why=''):
        if not route or self.navigator is None:
            self.stop_driving()
            self.outcome = 'EMPTY'
            return
        q = self.queued
        # A goal for the same Nav2 action preempts the running one without stopping
        # the boat; the other action is refused until the running goal is cancelled.
        preempt = (self.goal is not None and not self.goal.done and q is not None
                   and self.action(q['route'], q['precise']) == self.action(route, precise))
        if self.goal is not None and not self.goal.done:
            if preempt:  # it drives on until Nav2 takes the replacement: kept to cancel
                self.superseded.append(self.goal)
            else:
                self.goal.cancel()
                self.old = self.goal
        self.goal, self.outcome = None, None
        self.queued = {'route': list(route), 'precise': precise, 'why': why, 'tries': 0}
        self.driving()

    def drop_superseded(self):
        for goal in self.superseded:
            if not goal.done:
                goal.cancel()
        self.superseded = []

    def stop_driving(self):
        self.queued = None
        self.drop_superseded()
        if self.goal is not None and not self.goal.done:
            self.goal.cancel()
            self.old = self.goal
        self.goal = None

    def driving(self):
        if self.old is not None:
            if not self.old.done:
                return 'RUNNING'
            self.old = None
        now = self.board.now
        self.superseded = [goal for goal in self.superseded if not goal.done]
        if self.queued is not None and self.goal is None:
            q = self.queued
            self.goal = self.submit(q)
            q['tries'], q['sent'] = q['tries'] + 1, now
            if self.goal is None:
                self.drop_superseded()
                self.queued, self.outcome = None, 'REJECTED'
                return self.outcome
        if self.goal is None:
            return self.outcome
        if not self.goal.done:
            return 'RUNNING'
        status, q = self.goal.status, self.queued
        # Refused while another navigation winds down: rejected, or accepted and at
        # once aborted. Either way, send it again.
        refused = status == 'REJECTED' or (status == 'ABORTED' and q is not None
                                           and now - q['sent'] < self.limits['refusal_s'])
        # A goal that preempts one still ending can inherit its "Reached the goal": a
        # success at once and far from this route's end is that, not an arrival.
        pose = self.board.pose
        refused = refused or (status == 'SUCCEEDED' and q is not None and pose is not None
                              and now - q['sent'] < self.limits['refusal_s']
                              and math.dist(pose[:2], q['route'][-1][:2])
                              > self.limits['stale_success_m'])
        if refused and q is not None and q['tries'] < self.limits['goal_retries']:
            self.goal = None
            return self.driving()
        if refused:  # the replacement never took over: stop the old route
            self.drop_superseded()
        if (status == 'CANCELED' and not getattr(self.goal, 'cancel_requested', False)
                and self.preempt is not None):
            # Not the task's cancel: a human (the TUI revokes before the authority
            # record changes). Treat it as the override itself.
            self.preempt('goal cancelled outside the task')
        self.goal, self.queued, self.outcome = None, None, status
        return status

    def action(self, route, precise):
        """Which Nav2 action this leaf sends ``route`` to."""
        return action(route, precise)

    def submit(self, q):
        """Hand the queued route to Nav2 (``q`` as ``drive`` queued it)."""
        return self.navigator.send(q['route'], precise=q['precise'], quiet=q['tries'] > 0,
                                   why=q['why'])

    def keep_station(self, hold, precise=False):
        """Hold the pose this was first called at: drive back once ``hold`` metres off.

        ``precise`` drives back on the tight goal checker, so it ends well inside
        ``hold`` and the next drift has room before another re-goal.
        """
        pose = self.board.pose
        if pose is None:
            return
        if self.anchor is None:
            self.anchor = tuple(pose)
        if self.driving() == 'RUNNING':
            return
        if math.dist(pose[:2], self.anchor[:2]) > hold:
            self.drive([self.anchor], precise, why='hold station')
        elif hasattr(self.navigator, 'hold'):
            # No command for 0.5 s zeroes the thrusters and the boat drifts freely; a
            # zero velocity is held against the current by the velocity controller.
            self.navigator.hold()

    def terminate(self, new_status):
        self.stop_driving()


class Seen(Leaf):
    """Condition: SUCCESS if any of ``labels`` is tracked now, else FAILURE."""

    def __init__(self, name, labels):
        super().__init__(name)
        self.labels = tuple(labels)

    def update(self):
        return Status.SUCCESS if self.labelled(*self.labels) else Status.FAILURE


class PlanGates(Leaf):
    """Write the visible gates (nearest first) to ``key``; ``skip`` drops gates already passed."""

    def __init__(self, name, key, red, green, gates=2, skip=0, max_width=8.0):
        super().__init__(name, writes=(key,))
        self.key, self.red, self.green = key, red, green
        self.gates, self.skip, self.max_width = gates, skip, max_width

    def update(self):
        if self.board.pose is None:
            return Status.FAILURE
        route = geometry.gate_midpoints(self.labelled(self.red), self.labelled(self.green),
                                        self.board.pose, self.max_width)
        route = route[self.skip:self.skip + self.gates]
        if not route:
            self.feedback_message = 'no gate visible'
            return Status.FAILURE
        self.board.set(self.key, route, overwrite=True)
        return Status.SUCCESS


class PlanForward(Leaf):
    """Write a single waypoint ``distance`` metres ahead to ``key``."""

    def __init__(self, name, key, distance):
        super().__init__(name, writes=(key,))
        self.key, self.distance = key, distance

    def update(self):
        if self.board.pose is None:
            return Status.FAILURE
        waypoint = geometry.forward(self.board.pose, self.distance)
        self.board.set(self.key, [waypoint], overwrite=True)
        return Status.SUCCESS


class PlanCircle(Leaf):
    """Circle the nearest of ``labels``; ``clockwise`` is a bool or {label: bool}.

    Writes the first half to ``key``, the rest to ``key``_rest, and the nearest
    entry pose to ``key``_entry. Approach the entry before starting a full lap.
    """

    def __init__(self, name, key, labels, radius, clockwise, points=6, direction=None,
                 done=None, separation=1.5):
        """``direction`` is (key, {value: clockwise}): a colour read on the way wins.

        ``done`` names a key listing targets already circled (xy): those within
        ``separation`` are skipped. The chosen target's xy goes to ``key``_at.
        """
        super().__init__(name, reads=tuple(k for k in ((direction or (None,))[0], done) if k),
                         writes=(key, key + '_rest', key + '_target', key + '_entry',
                                 key + '_at'))
        self.key, self.labels, self.radius = key, tuple(labels), radius
        self.clockwise, self.points, self.direction = clockwise, points, direction
        self.done, self.separation = done, separation

    def turn(self, label):
        """Clockwise or not: the colour read under ``direction``, else the configured way."""
        if self.direction:
            key, ways = self.direction
            value = self.board.get(key) if self.board.exists(key) else None
            if value in ways:
                self.decide('circle', f'{value} indicator: '
                            + ('clockwise' if ways[value] else 'counter-clockwise'))
                return ways[value]
            self.decide('circle', 'no indicator read: the default way')
        return self.clockwise[label] if isinstance(self.clockwise, dict) else self.clockwise

    def update(self):
        pose = self.board.pose
        done = (self.board.get(self.done) or []) if self.done and self.board.exists(
            self.done) else []
        targets = [(label, x, y) for label, x, y in self.board.objects if label in self.labels
                   and all(math.dist((x, y), d) > self.separation for d in done)]
        if pose is None or not targets:
            return Status.FAILURE
        label, x, y = min(targets, key=lambda t: math.dist(t[1:], pose[:2]))
        clockwise = self.turn(label)
        self.board.set(self.key + '_at', (x, y), overwrite=True)
        poses = geometry.circle((x, y), self.radius, self.points, clockwise, pose[:2])
        # Two halves: Nav2 checks only the distance to a route's last pose, and a whole
        # circle ends where the boat starts it, so it would "arrive" at once.
        half = len(poses) // 2
        self.board.set(self.key, poses[:half], overwrite=True)
        self.board.set(self.key + '_rest', poses[half:], overwrite=True)
        self.board.set(self.key + '_target', label, overwrite=True)
        self.board.set(self.key + '_entry', [poses[-1]], overwrite=True)
        return Status.SUCCESS


class PlanReturn(Leaf):
    """Write ``source`` reversed (each waypoint turned around) to ``key``."""

    def __init__(self, name, key, source):
        super().__init__(name, reads=(source,), writes=(key,))
        self.key, self.source = key, source

    def update(self):
        self.board.set(self.key, geometry.reverse(self.board.get(self.source)), overwrite=True)
        return Status.SUCCESS


class PlanRoute(Leaf):
    """Write a fixed route of (forward, left) offsets from the current pose to ``key``.

    ``routes`` maps a tracked label to its route; the first label seen chooses.
    """

    def __init__(self, name, key, routes):
        super().__init__(name, writes=(key, key + '_choice'))
        self.key, self.routes = key, routes

    def update(self):
        if self.board.pose is None:
            return Status.FAILURE
        for label, offsets in self.routes.items():
            if self.labelled(label):
                self.board.set(self.key, geometry.relative(self.board.pose, offsets),
                               overwrite=True)
                self.board.set(self.key + '_choice', label, overwrite=True)
                return Status.SUCCESS
        return Status.FAILURE


class Navigate(Driver):
    """Drive the route in ``key`` with Nav2; cancels the goal if the tree preempts it.

    ``precise`` uses the tight goal checker (docking). A goal Nav2 gives up on is
    sent again up to ``retries`` times; with ``accept`` (m), ending that close to
    the route's end counts as arrived, since Nav2 may give up on the last metres.
    """

    def __init__(self, name, key, precise=False, retries=1, accept=None, why='', phase=None):
        super().__init__(name, reads=(key,))
        self.phase = phase or self.phase
        self.key, self.precise, self.retries, self.accept = key, precise, retries, accept
        self.why = why or name

    def initialise(self):
        super().initialise()
        self.failures = 0
        self.route = self.board.get(self.key) if self.board.exists(self.key) else None
        self.drive(self.route, self.precise, self.why)

    def update(self):
        state = self.driving()
        if state == 'RUNNING':
            return Status.RUNNING
        self.feedback_message = state or 'no route'
        if state == 'SUCCEEDED':
            return Status.SUCCESS
        if state in (None, 'EMPTY', 'CANCELED'):  # cancelled by someone else: not ours
            return Status.FAILURE
        pose = self.board.pose
        if self.accept and pose and math.dist(pose[:2], self.route[-1][:2]) <= self.accept:
            self.decide('accept', f'{self.name}: {state.lower()} but there')
            return Status.SUCCESS
        if self.failures < self.retries:
            self.failures += 1
            self.decide('retry', f'{self.name}: {state.lower()}, retry {self.failures}')
            self.retry()
            return Status.RUNNING
        return Status.FAILURE

    def retry(self):
        self.drive(self.route, self.precise, self.why)


class BackOut(Navigate):
    """Reverse straight to the route's end with Nav2's BackUp, keeping the heading.

    A goal behind the boat makes the rotation shim turn in place first, which jams
    it crosswise in a dock bay; reversing needs no turn, and the next goal turns
    once clear. ``speed`` (m/s), ``seconds`` (allowed), ``resends`` and ``pause`` come
    from the task YAML. BackUp stops at once on a collision ahead in the costmap, which
    is not a refusal to resend 30 times: the boat holds still ``pause`` seconds, for
    the costmap to settle, before each retry.
    """

    def __init__(self, name, key, speed, seconds, resends, pause, **kwargs):
        super().__init__(name, key, **kwargs)
        self.speed, self.seconds, self.resends, self.pause = speed, seconds, resends, pause

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.limits = dict(self.limits, goal_retries=self.resends)

    def initialise(self):
        self.paused_until = None
        super().initialise()

    def retry(self):
        self.paused_until = self.board.now + self.pause

    def update(self):
        if self.paused_until is None:
            return super().update()
        if self.board.now < self.paused_until:
            if hasattr(self.navigator, 'hold'):
                self.navigator.hold()  # sends nothing once a human has the boat
            return Status.RUNNING
        self.paused_until = None
        self.drive(self.route, self.precise, self.why)
        return Status.RUNNING

    def action(self, route, precise):
        return 'backup', False

    def submit(self, q):
        pose = self.board.pose
        distance = math.dist(pose[:2], q['route'][-1][:2]) if pose else 0.0
        return self.navigator.send(q['route'], quiet=q['tries'] > 0, why=q['why'],
                                   reverse=(distance, self.speed, self.seconds))


class Report(Leaf):
    """Publish a task event; ``include`` copies blackboard keys into it."""

    phase = 'report'

    def __init__(self, name, event, include=(), **fields):
        super().__init__(name, reads=include)
        self.event, self.include, self.fields = event, tuple(include), fields

    def update(self):
        details = dict(self.fields, **{key: self.board.get(key) for key in self.include})
        if self.board.pose is not None:
            details['pose'] = [round(value, 2) for value in self.board.pose]
        self.report(self.event, **details)
        return Status.SUCCESS


class ReportObjects(Leaf):
    """Report each newly tracked object with one of ``labels`` once; never finishes.

    Run it in parallel with the mission, so sightings are reported as they happen.
    """

    def __init__(self, name, event, labels, separation=1.0):
        super().__init__(name)
        self.event, self.labels, self.separation = event, tuple(labels), separation
        self.reported = []

    def update(self):
        for label, x, y in self.board.objects:
            if label in self.labels and not any(
                    seen == label and math.dist((x, y), at) < self.separation
                    for seen, at in self.reported):
                self.reported.append((label, (x, y)))
                self.report(self.event, label=label, position=[round(x, 2), round(y, 2)])
        return Status.RUNNING


class GateRun(Driver):
    """Pass gates one after another, in one flowing route, until ``gates`` are cleared.

    The route runs through every uncleared red/green gate in a chain ahead (up to
    ``lookahead``), so the boat does not stop between gates; it is re-sent only when
    the chain changes (a gate appears, or moves over ``replan`` metres). A gate is
    cleared when the hull crosses its line. Nav2 giving up is retried ``retries``
    times. With no gate ahead the camera sweeps the unexplored side, then the boat
    drives ``step`` metre legs up to ``search`` metres. The first gate is the nearest
    within ``first_radius``, any way. ``near``/``radius`` keep only
    gates within ``radius`` of a blackboard point (the black return gate by home);
    gates within ``exclude_radius`` of those in the ``exclude`` keys (other tasks'
    cleared gates) are skipped, and
    with ``toward`` (a label) each gate is crossed toward the nearest such object.
    ``red_side`` (port, starboard) fixes the way through each gate by its colour
    order, so a gate crossed the wrong way never counts; ``any`` takes either.
    ``corner_side`` (m) is when a turn between gates gets a mid-point, and
    ``exit_after`` (m) how far past each gate the route goes on (geometry.chain).
    While ``gates`` still owes one after a gate is cleared, the search goes on up to
    ``next_search`` metres (the next gate may stand beyond camera range); a run that
    ends short of ``gates`` fails, never claims success.
    The cleared gates go to ``key``; each is reported as ``gate_cleared``.
    """

    def __init__(self, name, key, red, green, gates=None, search=10.0, step=5.0,
                 max_width=8.0, min_width=1.0, near=None, radius=None, lookahead=3,
                 replan=0.7, retries=2, exclude=(), toward=None, red_side='any',
                 first_radius=None, exclude_radius=None, spread=None, corner_side=None,
                 exit_after=None, next_search=None):
        super().__init__(name, reads=((near,) if near else ()) + tuple(exclude),
                         writes=(key,))
        if red_side not in ('port', 'starboard', 'any'):
            raise ValueError(f'red_side must be port, starboard or any, not {red_side!r}')
        self.exclude, self.toward, self.red_side = tuple(exclude), toward, red_side
        self.first_radius, self.exclude_radius, self.spread = first_radius, exclude_radius, spread
        self.key, self.red, self.green, self.limit = key, red, green, gates
        self.search, self.step, self.max_width, self.min_width = search, step, max_width, min_width
        self.near, self.radius, self.lookahead = near, radius, lookahead
        self.replan, self.retries, self.corner_side = replan, retries, corner_side
        self.exit_after, self.next_search = exit_after, next_search

    def initialise(self):
        super().initialise()
        self.cleared, self.planned, self.failures = [], [], 0
        self.searched, self.looks, self.last, self.approached = 0.0, None, None, set()

    def visible_gates(self):
        reds = self.labelled(self.red)
        greens = reds if self.red == self.green else self.labelled(self.green)
        gates = geometry.gate_midpoints(reds, greens, self.board.pose, self.max_width,
                                        self.min_width, self.red_side)
        if self.near and self.board.exists(self.near):
            centre = self.board.get(self.near)
            if isinstance(centre, list):  # a one-pose route, such as the task's approach
                centre = centre[0]
            gates = [g for g in gates if centre and math.dist(g[:2], centre[:2]) <= self.radius]
        done = [g for key in self.exclude if self.board.exists(key)
                for g in self.board.get(key) or []]
        gates = geometry.unclaimed(gates, done, self.exclude_radius)
        marks = self.labelled(self.toward) if self.toward else []
        if marks:  # crossed toward the mark, whichever side the boat comes from
            gates = [g if math.cos(g[2] - geometry.heading(g, min(
                marks, key=lambda m: math.dist(m, g[:2])))) >= 0 else
                (g[0], g[1], math.atan2(-math.sin(g[2]), -math.cos(g[2]))) for g in gates]
        return gates

    def plan(self):
        """Return the chain of uncleared gates ahead, and whether it differs from the route sent.

        A planned gate stays planned while its markers drop out of sight (a lost
        track is not a lost gate); it moves only when re-seen over ``replan`` metres
        off. New gates are chained on past the last one, each the next ahead of it.
        """
        gates, plan, changed = self.visible_gates(), [], False
        for gate in self.planned:
            seen = min(gates, key=lambda g: math.dist(g[:2], gate[:2]), default=None)
            if seen is not None and self.replan < math.dist(seen[:2], gate[:2]) <= 2.0:
                if gate[:2] in self.approached:
                    self.approached.add(seen[:2])
                gate, changed = seen, True
            plan.append(gate)
        room = self.lookahead if self.limit is None else self.limit - len(self.cleared)
        origin = plan[-1] if plan else self.board.pose
        direction = plan[-1][2] if plan else self.cleared[-1][2] if self.cleared else None
        while len(plan) < room:
            if not plan and not self.cleared:  # a task's first gate: the nearest, any way
                gate = min((g for g in gates
                            if math.dist(g[:2], origin[:2]) <= self.first_radius),
                           key=lambda g: math.dist(g[:2], origin[:2]), default=None)
            else:
                gate = geometry.next_gate(gates, origin, self.cleared + plan, direction)
            if gate is None:
                break
            plan.append(gate)
            origin, direction, changed = gate, gate[2], True
        return plan, changed

    def update(self):
        pose = self.board.pose
        if pose is None:
            return Status.RUNNING
        last, self.last = self.last or pose, pose
        for gate in self.planned:  # a crossing counts only from the near side of the line
            if geometry.along(gate, pose) < 0:
                self.approached.add(gate[:2])
        for gate in self.planned:
            if (gate not in self.cleared and gate[:2] in self.approached
                    and geometry.crossed(gate, pose, last)):
                self.cleared.append(gate)
                self.board.set(self.key, list(self.cleared), overwrite=True)
                self.report('gate_cleared', gate=len(self.cleared),
                            position=[round(v, 2) for v in gate[:2]])
        self.planned = [g for g in self.planned if g not in self.cleared]
        if self.limit is not None and len(self.cleared) >= self.limit:
            return Status.SUCCESS
        chain, changed = self.plan()
        if chain and changed:
            self.phase = 'execute'
            if not self.planned:
                self.decide('execute', f'{len(chain)} gate(s) ahead: through the middle')
            self.planned, self.looks, self.searched = chain, None, 0.0
            route = geometry.chain(chain, self.corner_side, self.exit_after)
            if geometry.lined_up(chain[0], pose):  # past the lead-in: never turn back to it
                route = route[1:]
            self.drive(route, why=f'between gate {len(self.cleared) + 1} markers')
        state = self.driving()
        if state == 'RUNNING':
            return Status.RUNNING
        if self.planned and state is not None:  # ended, or arrived short of a gate
            state = 'SHORT' if state == 'SUCCEEDED' else state
            self.failures += 1
            if self.failures > self.retries or state == 'CANCELED':
                self.feedback_message = f'gate {len(self.cleared) + 1}: {state}'
                self.decide('give_up', f'gate {len(self.cleared) + 1} {state.lower()} '
                                       f'{self.failures}x: giving up')
                return Status.FAILURE
            self.decide('retry', f'gate {len(self.cleared) + 1} {state.lower()}: retry')
        self.planned = []  # re-send next tick, or the route ended past every gate
        if chain:
            return Status.RUNNING
        return self.explore(pose)

    def explore(self, pose):
        """No gate ahead: look along the least-seen bearings near the course, then drive on.

        Each look drives ``step`` metres along the bearing whose camera cone holds the
        most unexplored water within ``spread`` (rad) of the course, so the boat turns
        while moving; with no coverage grid it sweeps each side in place.
        """
        direction = self.cleared[-1][2] if self.cleared else pose[2]
        self.phase = 'search'
        coverage = self.board.coverage
        if self.looks is None:
            self.decide('search', 'no gate ahead: looking around')
            self.looks = 2 if coverage is not None else geometry.look_around(
                (*pose[:2], direction))
        reach = self.search
        if self.cleared and self.limit is not None and self.next_search is not None:
            reach = max(reach, self.next_search)  # owed a gate: it may be out of range
        if coverage is not None and self.looks and self.searched < reach:
            self.looks -= 1
            self.searched += self.step  # a look drives a leg too
            bearing = coverage.bearing_gaps(pose, self.board.fov, self.spread, direction)
            if bearing is not None:
                self.drive([(*geometry.forward((*pose[:2], bearing), self.step)[:2], bearing)],
                           why=f'look toward unexplored bearing {math.degrees(bearing):.0f}°')
                return Status.RUNNING
        elif self.looks:
            self.drive(self.looks.pop(0), why='sweep the camera for the next gate')
            return Status.RUNNING
        if self.searched < reach:
            self.searched += self.step
            self.looks = 1 if coverage is not None else []
            self.drive([geometry.forward((*pose[:2], direction), self.step)],
                       why='drive on to find the next gate')
            return Status.RUNNING
        self.feedback_message = f'{len(self.cleared)} gates, none ahead'
        if self.cleared and self.limit is not None and len(self.cleared) < self.limit:
            self.decide('give_up', f'{len(self.cleared)} of {self.limit} gates: '
                                   f'none ahead in {reach:.0f} m')
            return Status.FAILURE
        return Status.SUCCESS if self.cleared else Status.FAILURE


class Search(Driver):
    """Drive ahead up to ``distance`` metres in one leg until any of ``labels`` is tracked."""

    phase = 'search'

    def __init__(self, name, labels, distance):
        super().__init__(name)
        self.labels, self.distance = tuple(labels), distance

    def update(self):
        if self.labelled(*self.labels):
            return Status.SUCCESS
        state = self.driving()
        if state == 'RUNNING' or self.board.pose is None:
            return Status.RUNNING
        if state is None:
            self.decide('search', f'none of {self.labels[0]}..: driving ahead')
            self.drive([geometry.forward(self.board.pose, self.distance)],
                       why=f'look ahead for {self.labels[0]}')
            return Status.RUNNING
        self.feedback_message = f'none of {self.labels} within {self.distance} m'
        return Status.FAILURE


class CountObjects(Leaf):
    """Keep a running count of distinct ``label`` objects in ``key``; never finishes.

    Run it in parallel with the task; positions closer than ``separation`` are
    one object, so a track that is lost and found again is not counted twice.
    """

    def __init__(self, name, key, label, separation=1.5, beyond=None, horizon=30.0):
        super().__init__(name, reads=(beyond,) if beyond else (), writes=(key,))
        self.key, self.label, self.separation = key, label, separation
        self.beyond, self.horizon = beyond, horizon

    def initialise(self):
        self.points = []

    def in_task(self, point):
        """With ``beyond``, only objects past the first gate in that key, within ``horizon``."""
        if not self.beyond:
            return True
        gates = self.board.get(self.beyond) if self.board.exists(self.beyond) else None
        if not gates:
            return False
        x, y, yaw = gates[0]
        ahead = (point[0] - x) * math.cos(yaw) + (point[1] - y) * math.sin(yaw)
        return 0 < ahead and math.dist(point, (x, y)) <= self.horizon

    def update(self):
        seen = [point for point in self.labelled(self.label) if self.in_task(point)]
        self.points = geometry.unique(self.points + seen, self.separation)
        self.board.set(self.key, len(self.points), overwrite=True)
        return Status.RUNNING


class ReadColour(Leaf):
    """Read the indicator that sets a manoeuvre into ``key``, just before it is planned.

    ``labels`` maps a label to its value (``red_light_buoy: red``). The indicator read is
    the one nearest the segment from the first gate in ``gate`` to the nearest ``mark``
    and within ``reach`` of it, so another task's light is never taken for this one.
    Always succeeds: with no indicator in reach, ``key`` is cleared and the planner falls
    back to its configured way (and says so).
    """

    def __init__(self, name, key, labels, gate, marks, reach=8.0):
        super().__init__(name, reads=(gate,), writes=(key,))
        self.key, self.labels, self.gate = key, dict(labels), gate
        self.marks, self.reach = tuple(marks), reach

    def update(self):
        gates = self.board.get(self.gate) if self.board.exists(self.gate) else None
        marks = self.labelled(*self.marks)
        value = None
        if gates and marks:
            start = gates[0][:2]
            end = min(marks, key=lambda m: math.dist(m, start))
            lights = [(geometry.segment_distance((x, y), start, end), self.labels[label])
                      for label, x, y in self.board.objects if label in self.labels]
            lights = [light for light in lights if light[0] <= self.reach]
            if lights:
                value = min(lights)[1]
        self.board.set(self.key, value, overwrite=True)
        self.feedback_message = f'indicator {value or "not seen"}'
        return Status.SUCCESS


class Deliver(Leaf):
    """Deliver with the ``capability`` actuator at the nearest of ``labels``.

    The actuators come from the runner's setup (tasks/actuators.py); without a fitted,
    available one this fails at once ("capability off"), so a task that needs it is
    skipped. An actuated delivery without evidence still succeeds, reported as
    unverified; a failed one fails. The actuator is stopped whenever this ends.
    """

    def __init__(self, name, capability, labels, done='delivered'):
        super().__init__(name, reads=(done,), writes=(done,))
        self.capability, self.labels, self.done = capability, tuple(labels), done

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.actuator = (kwargs.get('actuators') or {}).get(self.capability)

    def initialise(self):
        self.target = None

    def update(self):
        if self.actuator is None or not self.actuator.available():
            self.feedback_message = f'capability off: {self.capability}'
            return Status.FAILURE
        if self.target is None:
            done = (self.board.get(self.done) if self.board.exists(self.done) else None) or []
            pose, targets = self.board.pose, [
                (label, x, y) for label, x, y in self.board.objects
                if label in self.labels and all(math.dist((x, y), d) > 1.5 for d in done)]
            if pose is None or not targets:
                return Status.FAILURE
            self.target = min(targets, key=lambda t: math.dist(t[1:], pose[:2]))
            self.actuator.start(self.target)
        state = self.actuator.state()
        if state == 'RUNNING':
            return Status.RUNNING
        if state in ('HIT', 'UNVERIFIED'):
            done = (self.board.get(self.done) if self.board.exists(self.done) else None) or []
            self.board.set(self.done, done + [self.target[1:]], overwrite=True)
            if self.report is not None:
                self.report('delivered', capability=self.capability, target=self.target[0],
                            position=list(self.target[1:]), verified=state == 'HIT')
            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        if getattr(self, 'actuator', None) is not None and self.target is not None:
            self.actuator.stop()


class ReportCount(Driver):
    """Report the count in ``key`` as ``event``; ``spin`` also turns once per object.

    RoboBoat accepts a count shown on the operator display (the event) or one
    360 degree turn in place per object, done before leaving the task. Each quarter
    turn is its own goal: Nav2 calls a route at the boat's own position done at once.
    """

    phase = 'report'

    def __init__(self, name, key, event, spin=False):
        super().__init__(name, reads=(key,))
        self.key, self.event, self.spin = key, event, spin

    def initialise(self):
        super().initialise()
        count = self.board.get(self.key) if self.board.exists(self.key) else 0
        self.report(self.event, count=count, method='spin' if self.spin else 'display')
        self.turns = []
        if self.spin and count and self.board.pose is not None:
            self.turns = [[pose] for pose in geometry.spin(self.board.pose, count)]

    def update(self):
        if self.driving() == 'RUNNING':
            return Status.RUNNING
        if not self.turns:
            return Status.SUCCESS
        self.drive(self.turns.pop(0), why='one turn per object counted')
        return Status.RUNNING


class PlanDock(Leaf):
    """Choose a bay and write it; with ``confirm``, check the bay already chosen.

    The first choice may rest on remembered tracks (a vessel out of view is soon
    forgotten, a light may have changed). At the staging pose, with the bay in view,
    a ``confirm`` choice that is not the bay being driven to writes the new one and
    fails, so the boat lines up on that one instead of driving into a taken bay.
    """

    def __init__(self, name, key, writes, confirm=False):
        super().__init__(name, writes=writes)
        self.key, self.confirm = key, confirm

    def changed(self, target):
        """Whether ``target`` (a banner xy) is not the bay chosen before (confirm only)."""
        before = (self.board.get(self.key) if self.confirm and self.board.exists(self.key)
                  else None)
        return before is not None and math.dist(before, target) > 0.5

    def write(self, target, values):
        changed = self.changed(target)
        for suffix, value in (('', target), *values):
            self.board.set(self.key + suffix, value, overwrite=True)
        if changed:
            self.feedback_message = 'the bay is no longer free: line up on another'
            return Status.FAILURE
        return Status.SUCCESS


class PlanBay(PlanDock):
    """Choose the nearest empty bay under the banner of the day and write its poses.

    Writes ``key`` + ``_staging`` and ``_berth`` (one-pose routes) and the
    chosen banner position to ``key``. A bay is occupied when a vessel is
    tracked within ``vessel_radius`` of its centre. ``confirm``: see PlanDock.
    """

    def __init__(self, name, key, banner, banners, vessels, berth=0.85, staging=3.0,
                 depth=1.5, vessel_radius=1.0, confirm=False):
        super().__init__(name, key, (key, key + '_staging', key + '_berth'), confirm)
        self.banner, self.banners, self.vessels = banner, tuple(banners), tuple(vessels)
        self.berth, self.staging, self.depth, self.vessel_radius = berth, staging, depth, \
            vessel_radius

    def update(self):
        pose = self.board.pose
        if pose is None:
            return Status.FAILURE
        everything = self.labelled(*self.banners)
        vessels = self.labelled(*self.vessels)
        options = []
        for target in self.labelled(self.banner):
            opening = geometry.dock_axis(everything, target, pose)
            poses = geometry.bay(target, opening, self.berth, self.staging, self.depth)
            if all(math.dist(v, poses['centre']) > self.vessel_radius for v in vessels):
                options.append((math.dist(target, pose[:2]), target, poses))
        if not options:
            self.feedback_message = f'no empty bay under {self.banner}'
            return Status.FAILURE
        _, target, poses = min(options, key=lambda option: option[0])
        return self.write(target, (('_staging', [poses['staging']]),
                                   ('_berth', [poses['berth']])))


class PlanSlip(PlanDock):
    """Choose the best free slip of a marina by its indicator and number; write its poses.

    Each indicator (``indicators``: label -> colour) belongs to the nearest number
    banner (``banners``: label -> number) within ``pair_radius``. The choice is the
    lowest-numbered free slip of the first colour in ``prefer`` that has one (2026:
    green, lowest number first), then the nearest. A slip is taken when a vessel is
    tracked within ``vessel_radius`` of its centre. The bay opens on the normal of
    the banners within ``pier_radius`` (one pier's spine), on the side toward the
    other pier (geometry.opening_side). Writes as PlanBay, plus ``key``_number,
    ``key``_colour and ``key``_slip (the slip's centre). ``confirm``: see PlanDock.
    """

    def __init__(self, name, key, indicators, banners, vessels, prefer=('green',),
                 berth=0.85, staging=3.0, depth=1.5, vessel_radius=1.0, pair_radius=1.0,
                 pier_radius=5.0, confirm=False):
        super().__init__(name, key, (key, key + '_staging', key + '_berth', key + '_number',
                                     key + '_colour', key + '_slip'), confirm)
        self.indicators, self.banners = dict(indicators), dict(banners)
        self.vessels, self.prefer = tuple(vessels), tuple(prefer)
        self.berth, self.staging, self.depth = berth, staging, depth
        self.vessel_radius, self.pair_radius, self.pier_radius = (
            vessel_radius, pair_radius, pier_radius)

    def slips(self, pose, vessels=()):
        """Every paired slip as (colour, number, banner xy, poses)."""
        banners = [(self.banners[label], (x, y)) for label, x, y in self.board.objects
                   if label in self.banners]
        spots = [b for _, b in banners]
        found = []
        for label, x, y in self.board.objects:
            if label not in self.indicators or not banners:
                continue
            number, banner = min(banners, key=lambda b: math.dist(b[1], (x, y)))
            if math.dist(banner, (x, y)) > self.pair_radius:
                continue
            pier = [b for b in spots if math.dist(b, banner) <= self.pier_radius]
            side = geometry.opening_side(pier, spots, vessels, pose)
            poses = geometry.bay(banner, geometry.dock_axis(pier, banner, side), self.berth,
                                 self.staging, self.depth)
            found.append((self.indicators[label], number, banner, poses))
        return found

    def update(self):
        pose = self.board.pose
        if pose is None:
            return Status.FAILURE
        vessels = self.labelled(*self.vessels)
        free = [s for s in self.slips(pose, vessels)
                if all(math.dist(v, s[3]['centre']) > self.vessel_radius for v in vessels)]
        for colour in self.prefer:
            options = [s for s in free if s[0] == colour]
            if options:
                break
        else:
            self.feedback_message = f'no free {"/".join(self.prefer)} slip in view'
            return Status.FAILURE
        colour, number, banner, poses = min(
            options, key=lambda s: (s[1], math.dist(s[2], pose[:2])))
        if not self.confirm or self.changed(banner):
            self.decide('slip', f'{colour} slip {number}'
                        + (': the chosen one is not free' if self.confirm else ''))
        return self.write(banner, (('_staging', [poses['staging']]),
                                   ('_berth', [poses['berth']]), ('_number', number),
                                   ('_colour', colour), ('_slip', poses['centre'])))


class Hold(Driver):
    """RUNNING for ``seconds`` of ROS time, then SUCCESS; with ``station``, keep station.

    Nav2 stops steering once a goal is reached, so the boat is commanded to zero
    velocity, which the velocity controller holds against the current; should it still
    drift ``station`` metres off, it drives back to where the hold began.
    """

    def __init__(self, name, seconds, station=None, precise=False):
        super().__init__(name)
        self.seconds, self.station, self.precise = seconds, station, precise

    def initialise(self):
        super().initialise()
        self.until = None

    def update(self):
        if self.until is None:
            self.until = self.board.now + self.seconds
        if self.board.now >= self.until:
            return Status.SUCCESS
        if self.station is not None:
            self.keep_station(self.station, self.precise)
        return Status.RUNNING


class PlanGatePass(Leaf):
    """Route through the last gate in ``source``; ``back`` reverses it.

    ``key``_end holds only the endpoint, for extending a completed crossing.
    """

    def __init__(self, name, key, source, back=False, before=1.5, after=1.1):
        super().__init__(name, reads=(source,), writes=(key, key + '_end'))
        self.key, self.source, self.back = key, source, back
        self.before, self.after = before, after

    def update(self):
        gates = self.board.get(self.source)
        if not gates:
            return Status.FAILURE
        route = geometry.through(gates[-1], self.before, self.after)
        if self.back:
            route = geometry.reverse(route)
        self.board.set(self.key, route, overwrite=True)
        self.board.set(self.key + '_end', [route[-1]], overwrite=True)
        return Status.SUCCESS


class Remember(Leaf):
    """Append the value under ``source`` to the list under ``key`` (e.g. targets done)."""

    def __init__(self, name, key, source):
        super().__init__(name, reads=(source, key), writes=(key,))
        self.key, self.source = key, source

    def update(self):
        done = list(self.board.get(self.key) or []) if self.board.exists(self.key) else []
        self.board.set(self.key, done + [self.board.get(self.source)], overwrite=True)
        return Status.SUCCESS


class PlanChainBack(Leaf):
    """Route back through every gate in ``source``, last first, each crossed the other way.

    The way out through gates the boat came in by (2026: the exit is the entry
    gates reversed); ``corner_side`` and ``after`` as for ``geometry.chain``.
    """

    def __init__(self, name, key, source, corner_side=1.0, after=1.1):
        super().__init__(name, reads=(source,), writes=(key,))
        self.key, self.source, self.corner_side, self.after = key, source, corner_side, after

    def update(self):
        gates = self.board.get(self.source) if self.board.exists(self.source) else None
        if not gates:
            self.feedback_message = f'no {self.source} gates to go back through'
            return Status.FAILURE
        back = [(x, y, geometry.wrap(yaw + math.pi)) for x, y, yaw in reversed(gates)]
        self.board.set(self.key, geometry.chain(back, self.corner_side, self.after),
                       overwrite=True)
        return Status.SUCCESS


class Deadline(Decorator):
    """Fail the child if it runs longer than ``seconds`` of ROS time.

    With ``limit_key``, a smaller limit found there when the child starts wins: the
    run's budget (trees/mission.py) keeps a task from eating the exit's time.
    """

    def __init__(self, name, child, seconds, limit_key=None):
        super().__init__(name=name, child=child)
        self.seconds, self.base, self.limit_key = seconds, seconds, limit_key
        self.board = self.attach_blackboard_client(name=name)
        self.board.register_key(key='now', access=Access.READ)
        if limit_key:
            self.board.register_key(key=limit_key, access=Access.READ)

    def initialise(self):
        self.started = None
        limit = (self.board.get(self.limit_key) if self.limit_key
                 and self.board.exists(self.limit_key) else None)
        self.seconds = self.base if limit is None else min(self.base, limit)

    def update(self):
        if self.started is None:
            self.started = self.board.now
        if self.board.now - self.started > self.seconds:
            self.feedback_message = f'over {self.seconds:g} s'
            self.decorated.stop(Status.INVALID)
            return Status.FAILURE
        return self.decorated.status


class SpeedCap(Decorator):
    """Cap Nav2's speed at ``speed`` m/s while the child runs; 0 leaves it uncapped.

    However the child ends (done, failed, stopped by an override), the cap is lifted.
    """

    def __init__(self, name, child, speed):
        super().__init__(name=name, child=child)
        self.speed = speed

    def setup(self, **kwargs):
        self.navigator = kwargs.get('navigator')

    def initialise(self):
        self.cap(self.speed)

    def update(self):
        return self.decorated.status

    def terminate(self, new_status):
        self.cap(0.0)

    def cap(self, speed):
        if self.speed and hasattr(self.navigator, 'speed_limit'):
            self.navigator.speed_limit(speed)


class PlanHome(Leaf):
    """Pose ``approach`` metres out from home, facing home.

    With no approach, restore the initial position and heading instead.
    """

    def __init__(self, name, key, approach=None):
        super().__init__(name, writes=(key,))
        self.key, self.approach = key, approach

    def update(self):
        home, pose = self.board.home, self.board.pose
        if home is None or pose is None:
            return Status.FAILURE
        if self.approach is None:
            self.board.set(self.key, [home], overwrite=True)
            return Status.SUCCESS
        out = geometry.heading(home, pose)
        point = (home[0] + self.approach * math.cos(out), home[1] + self.approach * math.sin(out))
        self.board.set(self.key, [(*point, geometry.heading(point, home))], overwrite=True)
        return Status.SUCCESS


class PlanPose(Leaf):
    """Write the fixed map pose ``pose`` (x, y, yaw) to ``key`` as a one-pose route."""

    def __init__(self, name, key, pose):
        super().__init__(name, writes=(key,))
        self.key, self.pose = key, tuple(float(value) for value in pose)

    def update(self):
        self.board.set(self.key, [self.pose], overwrite=True)
        return Status.SUCCESS


class PlanLook(Leaf):
    """Write an in-place sweep: one side to ``key``, the other to ``key``_rest.

    ``angle`` (rad) is how far each way; ``key``_back faces the way the boat faced.
    """

    def __init__(self, name, key, angle=1.05):
        super().__init__(name, writes=(key, key + '_rest', key + '_back'))
        self.key, self.angle = key, angle

    def update(self):
        if self.board.pose is None:
            return Status.FAILURE
        self.board.set(self.key + '_back', [tuple(self.board.pose)], overwrite=True)
        first, rest = geometry.look_around(self.board.pose, self.angle)
        self.board.set(self.key, first, overwrite=True)
        self.board.set(self.key + '_rest', rest, overwrite=True)
        return Status.SUCCESS


class PlanConcat(Leaf):
    """Write the routes in ``sources`` joined end to end to ``key``: one goal, no stops."""

    def __init__(self, name, key, sources):
        super().__init__(name, reads=tuple(sources), writes=(key,))
        self.key, self.sources = key, tuple(sources)

    def update(self):
        route = [pose for source in self.sources for pose in (self.board.get(source) or [])]
        if not route:
            return Status.FAILURE
        self.board.set(self.key, route, overwrite=True)
        return Status.SUCCESS


class PlanHypothesis(Leaf):
    """Write a view of the recognised ``kind`` of task to ``key``: ``standoff`` m before it.

    The runner keeps the recogniser's hypotheses on the blackboard. With none of that
    kind, or the boat already within ``2 * standoff``, there is nothing to approach.
    """

    def __init__(self, name, key, kind, standoff=6.0):
        super().__init__(name, reads=('hypotheses',), writes=(key,))
        self.key, self.kind, self.standoff = key, kind, standoff

    def update(self):
        pose = self.board.pose
        found = [h for h in (self.board.get('hypotheses') if self.board.exists('hypotheses')
                             else None) or []
                 if h['type'] == self.kind and h['state'] in ('hypothesis', 'confirmed')]
        if pose is None or not found:
            return Status.FAILURE
        best = max(found, key=lambda h: h['confidence'])
        x, y, yaw = best['pose']
        if math.dist(pose[:2], (x, y)) < 2 * self.standoff:
            return Status.FAILURE
        view = (x - self.standoff * math.cos(yaw), y - self.standoff * math.sin(yaw), yaw)
        self.board.set(self.key, [view], overwrite=True)
        self.decide('recognise', f'{self.kind} seen at ({x:.0f}, {y:.0f}): heading there')
        return Status.SUCCESS


class PlanView(Leaf):
    """Write a view of the nearest ``label``, ``standoff`` m off on the boat's side, to ``key``."""

    def __init__(self, name, key, label, standoff=8.0, pair=None):
        """``pair`` ({label, with, min, max}): only a ``label`` beside its companions."""
        super().__init__(name, writes=(key,))
        self.key, self.label, self.standoff, self.pair = key, label, standoff, pair

    def update(self):
        pose, targets = self.board.pose, self.labelled(self.label)
        if self.pair:
            targets = [(x, y) for _, x, y in geometry.paired(self.board.objects, self.pair)]
        if pose is None or not targets:
            return Status.FAILURE
        target = min(targets, key=lambda t: math.dist(t, pose[:2]))
        away = geometry.heading(target, pose)
        point = (target[0] + self.standoff * math.cos(away),
                 target[1] + self.standoff * math.sin(away))
        self.board.set(self.key, [(*point, geometry.heading(point, target))], overwrite=True)
        return Status.SUCCESS
