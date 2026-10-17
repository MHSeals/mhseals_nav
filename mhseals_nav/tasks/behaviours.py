"""Reusable task leaves. Each reads and writes the blackboard; only Navigate talks to Nav2.

Blackboard keys filled by the runner every tick:
  objects  tracked objects as (label, x, y) in the map frame
  pose     the boat's (x, y, yaw) in the map frame, or None
  now      ROS time in seconds
  home     where the run started (x, y, yaw), for the return
Leaves pass routes to each other through keys named in their arguments.
"""

import math

import py_trees
from py_trees.common import Access, Status
from py_trees.decorators import Decorator

from mhseals_nav.tasks import geometry


class Leaf(py_trees.behaviour.Behaviour):
    """A behaviour with a blackboard client for the shared world keys."""

    def __init__(self, name, reads=(), writes=()):
        super().__init__(name)
        self.board = self.attach_blackboard_client(name=name)
        for key in ('objects', 'pose', 'now', 'home', *reads):
            self.board.register_key(key=key, access=Access.READ)
        for key in writes:
            self.board.register_key(key=key, access=Access.WRITE)

    def labelled(self, *labels):
        return [(x, y) for label, x, y in self.board.objects if label in labels]


class WaitFor(Leaf):
    """RUNNING until at least ``count`` objects with each of ``labels`` are tracked."""

    def __init__(self, name, labels, count=1, any_of=False):
        super().__init__(name)
        self.labels, self.count, self.any_of = tuple(labels), count, any_of

    def update(self):
        seen = [len(self.labelled(label)) >= self.count for label in self.labels]
        return Status.SUCCESS if (any(seen) if self.any_of else all(seen)) else Status.RUNNING


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
    """Circle the nearest of ``labels``; ``clockwise`` is a bool or {label: bool}."""

    def __init__(self, name, key, labels, radius, clockwise, points=6):
        super().__init__(name, writes=(key, key + '_target'))
        self.key, self.labels, self.radius = key, tuple(labels), radius
        self.clockwise, self.points = clockwise, points

    def update(self):
        pose = self.board.pose
        targets = [(label, x, y) for label, x, y in self.board.objects if label in self.labels]
        if pose is None or not targets:
            return Status.FAILURE
        label, x, y = min(targets, key=lambda t: math.dist(t[1:], pose[:2]))
        clockwise = (self.clockwise[label] if isinstance(self.clockwise, dict)
                     else self.clockwise)
        self.board.set(self.key, geometry.circle((x, y), self.radius, self.points, clockwise,
                                                 pose[:2]), overwrite=True)
        self.board.set(self.key + '_target', label, overwrite=True)
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


class Navigate(Leaf):
    """Drive the route in ``key`` with Nav2; cancels the goal if the tree preempts it.

    ``precise`` uses the tight goal checker (docking), ``key_index`` drives one
    pose of the route.
    """

    def __init__(self, name, key, precise=False):
        super().__init__(name, reads=(key,))
        self.key, self.precise, self.navigator, self.goal = key, precise, None, None

    def setup(self, **kwargs):
        self.navigator = kwargs['navigator']

    def initialise(self):
        self.goal = self.navigator.send(self.board.get(self.key), precise=self.precise)

    def update(self):
        if self.goal is None:
            return Status.FAILURE  # Nav2 rejected the goal
        if not self.goal.done:
            return Status.RUNNING
        self.feedback_message = self.goal.status
        return Status.SUCCESS if self.goal.status == 'SUCCEEDED' else Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.goal is not None:
            self.goal.cancel()


class Report(Leaf):
    """Publish a task event; ``include`` copies blackboard keys into it."""

    def __init__(self, name, event, include=(), **fields):
        super().__init__(name, reads=include)
        self.event, self.include, self.fields = event, tuple(include), fields

    def setup(self, **kwargs):
        self.report = kwargs['report']

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

    def setup(self, **kwargs):
        self.report = kwargs['report']

    def update(self):
        for label, x, y in self.board.objects:
            if label in self.labels and not any(
                    seen == label and math.dist((x, y), at) < self.separation
                    for seen, at in self.reported):
                self.reported.append((label, (x, y)))
                self.report(self.event, label=label, position=[round(x, 2), round(y, 2)])
        return Status.RUNNING


class GateRun(Leaf):
    """Pass gates one after another until ``gates`` are cleared or none is ahead.

    Each gate is the nearest uncleared red/green pair ahead; the boat is driven
    across its line, not just to it. With no gate in sight it drives ahead in
    ``step`` metre legs, up to ``search`` metres, and turns toward a gate the
    moment one appears. ``near``/``radius`` keep only gates within ``radius``
    of a blackboard point (the black return gate by home). The cleared gates go
    to ``key``; each is reported as ``gate_cleared``.
    """

    def __init__(self, name, key, red, green, gates=None, search=10.0, step=5.0,
                 max_width=8.0, min_width=1.0, near=None, radius=None):
        super().__init__(name, reads=(near,) if near else (), writes=(key,))
        self.key, self.red, self.green, self.limit = key, red, green, gates
        self.search, self.step, self.max_width, self.min_width = search, step, max_width, min_width
        self.near, self.radius = near, radius

    def setup(self, **kwargs):
        self.navigator, self.report = kwargs['navigator'], kwargs['report']

    def initialise(self):
        self.cleared, self.goal, self.target, self.searched = [], None, None, 0.0
        self.cancelled = None

    def visible_gate(self):
        reds = self.labelled(self.red)
        greens = reds if self.red == self.green else self.labelled(self.green)
        gates = geometry.gate_midpoints(reds, greens, self.board.pose, self.max_width,
                                        self.min_width)
        if self.near:
            centre = self.board.get(self.near)
            gates = [g for g in gates if centre and math.dist(g[:2], centre[:2]) <= self.radius]
        direction = self.cleared[-1][2] if self.cleared else None
        return geometry.next_gate(gates, self.board.pose, self.cleared, direction)

    def update(self):
        if self.board.pose is None:
            return Status.RUNNING
        # Nav2 takes one navigation at a time: let a cancelled goal finish first.
        if self.cancelled is not None and not self.cancelled.done:
            return Status.RUNNING
        self.cancelled = None
        if self.goal is not None:
            if self.target is None and self.visible_gate() is not None:
                self.goal.cancel()  # a gate appeared while searching: go to it
                self.cancelled, self.goal = self.goal, None
                return Status.RUNNING
            elif not self.goal.done:
                return Status.RUNNING
            elif self.target is not None:
                # The boat's own pose decides: Nav2 may give up on the last metres past
                # a gate it has already carried the hull through.
                if (self.goal.status != 'SUCCEEDED' and
                        not geometry.crossed(self.target, self.board.pose)):
                    self.feedback_message = f'gate {len(self.cleared) + 1}: {self.goal.status}'
                    return Status.FAILURE
                self.cleared.append(self.target)
                self.board.set(self.key, list(self.cleared), overwrite=True)
                self.report('gate_cleared', gate=len(self.cleared),
                            position=[round(v, 2) for v in self.target[:2]])
                self.goal = self.target = None
            else:
                self.goal = None
        if self.limit is not None and len(self.cleared) >= self.limit:
            return Status.SUCCESS
        gate = self.visible_gate()
        if gate is not None:
            self.target, self.searched = gate, 0.0
            self.goal = self.navigator.send(geometry.through(gate))
            return Status.RUNNING if self.goal is not None else Status.FAILURE
        if self.searched < self.search:
            self.searched += self.step
            self.goal = self.navigator.send([geometry.forward(self.board.pose, self.step)])
            return Status.RUNNING if self.goal is not None else Status.FAILURE
        self.feedback_message = f'{len(self.cleared)} gates, none ahead'
        return Status.SUCCESS if self.cleared else Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.goal is not None:
            self.goal.cancel()


class Search(Leaf):
    """Drive ahead in legs until any of ``labels`` is tracked, or ``distance`` runs out."""

    def __init__(self, name, labels, distance, step=5.0):
        super().__init__(name)
        self.labels, self.distance, self.step = tuple(labels), distance, step

    def setup(self, **kwargs):
        self.navigator = kwargs['navigator']

    def initialise(self):
        self.goal, self.driven = None, 0.0

    def update(self):
        if self.labelled(*self.labels):
            if self.goal is not None:
                self.goal.cancel()
                if not self.goal.done:  # Nav2 takes one navigation at a time
                    return Status.RUNNING
                self.goal = None
            return Status.SUCCESS
        if self.board.pose is None or (self.goal is not None and not self.goal.done):
            return Status.RUNNING
        if self.driven >= self.distance:
            self.feedback_message = f'none of {self.labels} within {self.distance} m'
            return Status.FAILURE
        self.driven += self.step
        self.goal = self.navigator.send([geometry.forward(self.board.pose, self.step)])
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.goal is not None:
            self.goal.cancel()


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


class ReportCount(Leaf):
    """Report the count in ``key`` as ``event``; ``spin`` also turns once per object.

    RoboBoat accepts a count shown on the operator display (the event) or one
    360 degree turn in place per object, done before leaving the task.
    """

    def __init__(self, name, key, event, spin=False):
        super().__init__(name, reads=(key,))
        self.key, self.event, self.spin = key, event, spin

    def setup(self, **kwargs):
        self.navigator, self.report = kwargs['navigator'], kwargs['report']

    def initialise(self):
        self.goal = None
        count = self.board.get(self.key) if self.board.exists(self.key) else 0
        self.report(self.event, count=count, method='spin' if self.spin else 'display')
        if self.spin and count and self.board.pose is not None:
            self.goal = self.navigator.send(geometry.spin(self.board.pose, count))

    def update(self):
        if self.goal is None or self.goal.done:
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.goal is not None:
            self.goal.cancel()


class PlanBay(Leaf):
    """Choose the nearest empty bay under the banner of the day and write its poses.

    Writes ``key`` + ``_staging`` and ``_berth`` (one-pose routes) and the
    chosen banner position to ``key``. A bay is occupied when a vessel is
    tracked within ``vessel_radius`` of its centre.
    """

    def __init__(self, name, key, banner, banners, vessels, berth=0.85, staging=3.0,
                 depth=1.5, vessel_radius=1.0):
        super().__init__(name, writes=(key, key + '_staging', key + '_berth'))
        self.key, self.banner, self.banners, self.vessels = key, banner, tuple(banners), \
            tuple(vessels)
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
        self.board.set(self.key, target, overwrite=True)
        self.board.set(self.key + '_staging', [poses['staging']], overwrite=True)
        self.board.set(self.key + '_berth', [poses['berth']], overwrite=True)
        return Status.SUCCESS


class Hold(Leaf):
    """RUNNING for ``seconds`` of ROS time, then SUCCESS."""

    def __init__(self, name, seconds):
        super().__init__(name)
        self.seconds = seconds

    def initialise(self):
        self.until = None

    def update(self):
        if self.until is None:
            self.until = self.board.now + self.seconds
        return Status.SUCCESS if self.board.now >= self.until else Status.RUNNING


class PlanGatePass(Leaf):
    """Write a route through the last gate in ``source``; ``back`` crosses it the other way."""

    def __init__(self, name, key, source, back=False):
        super().__init__(name, reads=(source,), writes=(key,))
        self.key, self.source, self.back = key, source, back

    def update(self):
        gates = self.board.get(self.source)
        if not gates:
            return Status.FAILURE
        route = geometry.through(gates[-1])
        self.board.set(self.key, geometry.reverse(route) if self.back else route, overwrite=True)
        return Status.SUCCESS


class Deadline(Decorator):
    """Fail the child if it runs longer than ``seconds`` of ROS time."""

    def __init__(self, name, child, seconds):
        super().__init__(name=name, child=child)
        self.seconds = seconds
        self.board = self.attach_blackboard_client(name=name)
        self.board.register_key(key='now', access=Access.READ)

    def initialise(self):
        self.started = None

    def update(self):
        if self.started is None:
            self.started = self.board.now
        if self.board.now - self.started > self.seconds:
            self.feedback_message = f'over {self.seconds:g} s'
            self.decorated.stop(Status.INVALID)
            return Status.FAILURE
        return self.decorated.status


class PlanHome(Leaf):
    """Write a pose ``approach`` metres out from home, on the boat's side, facing home."""

    def __init__(self, name, key, approach):
        super().__init__(name, writes=(key,))
        self.key, self.approach = key, approach

    def update(self):
        home, pose = self.board.home, self.board.pose
        if home is None or pose is None:
            return Status.FAILURE
        out = geometry.heading(home, pose)
        point = (home[0] + self.approach * math.cos(out), home[1] + self.approach * math.sin(out))
        self.board.set(self.key, [(*point, geometry.heading(point, home))], overwrite=True)
        return Status.SUCCESS
