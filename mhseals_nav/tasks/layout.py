"""The course-layout belief: where each task not found yet most likely is, and its cost.

Built from the handbook and rotation-rule priors and from every observation so far:
- prior: each task's placement ``regions`` (its search boxes in the start frame, a
  mirrored box counting on both sides until a side cue fixes the course side S);
- found: a task's own labels (the ones its ``tasks`` keys name) seen inside its regions
  place it there;
- not found: the cells the camera has looked at (the run's coverage grid, any task's
  looks) keep only ``miss`` of their weight, so the belief moves to what is still unseen.

The mission plans with it: the cost of a task from a place is the travel to where the
task likely is, a search that grows with how spread out that guess still is, and the
task's own ``service_s`` (both timed at ``transit_mps``, scaled by the speed measured so
far); the order of the ``agenda`` tasks is the one that fits the most points into the
time left, the way out from the last one counted (``plan``), and the exit's reserve
after each task is what the way out from there needs (``reserve``). The search heads
for the guess first (``HeadFor``). No ROS, no py_trees: numpy only.
"""

import itertools
import math

import numpy as np


class Guess:
    """Where a task likely is: ``xy`` (m), ``spread`` (m, 0 once found), ``found``."""

    def __init__(self, xy, spread, found):
        self.xy, self.spread, self.found = tuple(xy), float(spread), found


def side_boxes(regions, side):
    """Yield (box, weight) of ``regions`` on the course side: both while ``side`` is None.

    A region is {box: [[x0, y0], [x1, y1]], weight, mirror}; ``mirror`` also counts the
    box mirrored across the start line (y -> -y), and a known ``side`` (port: y > 0)
    keeps only the one on that side. Each box is ((x0, y0), (x1, y1)), lowest first.
    """
    for region in regions or ():
        (x0, y0), (x1, y1) = region['box']
        boxes = [(y0, y1)]
        if region.get('mirror'):
            boxes.append((-y1, -y0))
            if side is not None:
                boxes = [b for b in boxes if (b[0] + b[1] > 0) == (side == 'port')]
        for lo, hi in boxes:
            yield ((min(x0, x1), min(lo, hi)), (max(x0, x1), max(lo, hi))), region.get(
                'weight', 1.0)


def inside(xy, regions, side):
    """Tell whether ``xy`` lies in one of ``regions`` on the course side."""
    return any(x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1
               for ((x0, y0), (x1, y1)), _ in side_boxes(regions, side))


class Layout:
    """The belief over the mission's tasks; ``params`` is the mission's (its ``layout``)."""

    def __init__(self, params):
        cfg = params['layout']
        self.miss, self.transit = float(cfg['miss']), float(cfg['transit_mps'])
        self.search_s_per_m = float(cfg['search_s_per_m'])
        margin = cfg['exit_margin_s']
        self.exit_margin_s = None if margin is None else float(margin)
        self.exit_mps, self.exit_factor = float(cfg['exit_mps']), float(cfg['exit_factor'])
        self.exit_floor_s, self.exit_pad_m = float(cfg['exit_floor_s']), float(cfg['exit_pad_m'])
        self.measure_m, self.moving_mps = float(cfg['measure_m']), float(cfg['moving_mps'])
        self.last, self.moved, self.moving = None, 0.0, 0.0
        self.agenda = list(params.get('agenda') or ())
        self.tasks = {}
        for name, keys in cfg['tasks'].items():
            task = params[name]
            # The labels the task's own keys name: a label, a list, or a map's keys.
            labels = [label for key in keys for label in (
                [task[key]] if isinstance(task.get(key), str) else task.get(key) or ())]
            self.tasks[name] = {'regions': (task.get('search') or {}).get('regions') or [],
                                'labels': tuple(labels), 'service_s': float(task['service_s']),
                                'pad': float(task.get('circle_radius') or 0.0) + self.exit_pad_m}

    def observe(self, pose, now):
        """Measure the boat's speed while it moves (the speed policy's, whatever it is)."""
        if self.last is not None and now > self.last[2]:
            step, dt = math.dist(pose[:2], self.last[:2]), now - self.last[2]
            if step / dt >= self.moving_mps:
                self.moved, self.moving = self.moved + step, self.moving + dt
        self.last = (pose[0], pose[1], now)

    @property
    def speed(self):
        """Return the measured speed once ``measure_m`` were driven, else ``transit_mps``."""
        return self.moved / self.moving if self.moved >= self.measure_m else self.transit

    def grid(self, name, coverage, side):
        """Return the task's weight per coverage cell: its prior, less what was looked at."""
        weight = coverage.prior(self.tasks[name]['regions'], side, floor=0.0)
        return np.where(np.isfinite(coverage.seen), weight * self.miss, weight)

    def guess(self, name, coverage, tracks, side, min_confidence=0.5):
        """Return the task's Guess, or None for a task the layout has no prior for."""
        task = self.tasks.get(name)
        if task is None:
            return None
        seen = [(t.x, t.y) for t in tracks or () if t.label in task['labels'] and (
            not t.prior and t.confidence >= min_confidence) and inside(
                (t.x, t.y), task['regions'], side)]
        if seen:
            return Guess(np.mean(seen, axis=0), 0.0, True)
        weight = self.grid(name, coverage, side)
        total = weight.sum()
        if total <= 0:
            return None
        gx, gy = np.meshgrid(coverage.xs, coverage.ys, indexing='ij')
        mx, my = (weight * gx).sum() / total, (weight * gy).sum() / total
        spread = math.sqrt((weight * ((gx - mx) ** 2 + (gy - my) ** 2)).sum() / total)
        return Guess((mx, my), spread, False)

    def cost(self, name, start, guess, expected_s):
        """Return (seconds, where it ends) of the task from ``start`` (x, y).

        A task with no guess costs its mission ``expected_s`` and ends where it started.
        """
        if guess is None:
            return float(expected_s), tuple(start[:2])
        # Search and service were timed at transit_mps: they scale with the boat's speed.
        scale = self.transit / self.speed
        return (math.dist(start[:2], guess.xy) / self.speed + scale * (
            self.search_s_per_m * guess.spread + self.tasks[name]['service_s']), guess.xy)

    def reserve(self, name, end, via, cap, side=None):
        """Return the exit's reserve after task ``name`` ending at ``end``.

        Without ``exit_margin_s``, or for a task with no regions on the course ``side``,
        it is ``cap`` (the fixed ``exit_reserve``). Otherwise it is sized from the worst
        place the boat can leave from: the farthest (by the way out through ``via``: the
        entry gates, last first, then home) of ``end`` (the guess, or the boat's own
        position once known) and every corner of the task's regions on the course
        ``side`` (both sides while unknown), each box padded by the task's ``pad`` (its
        circle radius plus ``exit_pad_m``: a turn round a mark on the box edge, a gate
        reached from behind). That way at the exit's speed (the slower of the measured
        ``speed`` and ``exit_mps``: the mean leaves out stops and the exit may be slower,
        a lower vx_max or a headwind), times ``exit_factor``, plus ``exit_margin_s``, at
        least ``exit_floor_s``; never capped.
        """
        task = self.tasks.get(name) or {}
        pad = task.get('pad', 0.0)
        corners = [(x, y) for ((x0, y0), (x1, y1)), _ in side_boxes(task.get('regions'), side)
                   for x in (x0 - pad, x1 + pad) for y in (y0 - pad, y1 + pad)]
        if self.exit_margin_s is None or not corners:
            return cap
        way = max(self.way_out(xy, via, 1.0) for xy in [tuple(end[:2]), *corners])
        return max(self.exit_floor_s, self.exit_factor * way / self.exit_speed
                   + self.exit_margin_s)

    @property
    def exit_speed(self):
        """Return the speed the exit's reserve is timed at: measured, at most exit_mps."""
        return min(self.speed, self.exit_mps)

    def way_out(self, xy, via, speed=None):
        """Return the seconds from ``xy`` home through ``via`` at ``speed`` (the boat's)."""
        way, here = 0.0, tuple(xy[:2])
        for point in via:
            way, here = way + math.dist(here, point[:2]), tuple(point[:2])
        return way / (speed or self.speed)

    def plan(self, names, start, via, left_s, guesses, rows, cap, side=None):
        """Return the best order of ``names``: {name: (seconds, exit reserve after it)}.

        Every order is tried (an agenda is a few tasks): each task in turn is kept when
        its cost from where the last one ended and the exit's ``reserve`` from where it
        ends still fit ``left_s``; the order with the most points wins, then the least
        time including the way out, then the mission's order. ``rows`` maps a name to its
        budget row (value, expected_s); a task worth no points is never planned.
        """
        best = None
        for rank, order in enumerate(itertools.permutations(names)):
            here, used, points, kept = tuple(start[:2]), 0.0, 0.0, {}
            for name in order:
                row = rows[name]
                if row['value'] <= 0:
                    continue
                seconds, end = self.cost(name, here, guesses.get(name), row['expected_s'])
                reserve = self.reserve(name, end, via, cap, side)
                if used + seconds + reserve <= left_s:
                    used, points, here = used + seconds, points + row['value'], end
                    kept[name] = (seconds, reserve)
            # The way out itself, not the reserve: a fixed reserve would tie every order.
            key = (-points, used + self.way_out(here, via), rank)
            if best is None or key < best[0]:
                best = (key, kept)
        return best[1] if best else {}
