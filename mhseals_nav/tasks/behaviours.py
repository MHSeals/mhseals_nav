"""Reusable task leaves. Each reads and writes the blackboard; only Navigate talks to Nav2.

Blackboard keys filled by the runner every tick:
  objects  tracked objects as (label, x, y) in the map frame
  pose     the boat's (x, y, yaw) in the map frame, or None
  now      ROS time in seconds
Leaves pass routes to each other through keys named in their arguments.
"""

import math

import py_trees
from py_trees.common import Access, Status

from mhseals_nav.tasks import geometry


class Leaf(py_trees.behaviour.Behaviour):
    """A behaviour with a blackboard client for the shared world keys."""

    def __init__(self, name, reads=(), writes=()):
        super().__init__(name)
        self.board = self.attach_blackboard_client(name=name)
        for key in ('objects', 'pose', 'now', *reads):
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
    """Drive the route in ``key`` with Nav2; cancels the goal if the tree preempts it."""

    def __init__(self, name, key):
        super().__init__(name, reads=(key,))
        self.key, self.navigator, self.goal = key, None, None

    def setup(self, **kwargs):
        self.navigator = kwargs['navigator']

    def initialise(self):
        self.goal = self.navigator.send(self.board.get(self.key))

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
