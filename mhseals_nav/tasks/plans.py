"""Planning leaves: each writes a route (a list of (x, y, yaw) poses) to a blackboard key.

They read the shared world keys (see tasks.behaviours) and never drive; Navigate drives
what they write.
"""

import math

from mhseals_nav.tasks import geometry
from mhseals_nav.tasks.behaviours import Leaf
from py_trees.common import Status


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


class PlanGatePass(Leaf):
    """Route through the last gate in ``source``; ``back`` reverses it.

    ``key``_end holds only the endpoint, for extending a completed crossing.
    """

    def __init__(self, name, key, source, back=False, before=1.5, *, after):
        super().__init__(name, reads=(source,), writes=(key, key + '_end'))
        self.key, self.source, self.back = key, source, back
        self.before, self.after = before, after

    def update(self):
        gates = self.board.get(self.source)
        if not gates:
            return Status.FAILURE
        route = geometry.through(gates[-1], self.before, after=self.after)
        if self.back:
            route = geometry.reverse(route)
        self.board.set(self.key, route, overwrite=True)
        self.board.set(self.key + '_end', [route[-1]], overwrite=True)
        return Status.SUCCESS


class PlanChainBack(Leaf):
    """Route back through every gate in ``source``, last first, each crossed the other way.

    The way out through gates the boat came in by (2026: the exit is the entry
    gates reversed); ``corner_side`` and ``after`` as for ``geometry.chain``.
    """

    def __init__(self, name, key, source, corner_side, after):
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
