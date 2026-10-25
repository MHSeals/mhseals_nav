"""Purposeful search for task elements not seen yet, or seen too uncertainly.

``search(name, labels, cfg)`` tries the cheapest strategy first and stops the moment
the wanted labels are tracked live:
  LookAtPriors  look where the course map, or a weak track, says the thing is
  Scan          turn toward the least-seen bearing ahead while moving on
  NextView      next-best-view by information gain over the camera coverage grid
  Sweep         a bounded lawnmower or expanding square: the guaranteed fallback
Goals are re-planned only when one ends or nears its end, or when another view is
clearly (``hysteresis``) better, so the boat turns while moving and never dithers.
"""

import math
from pathlib import Path

from mhseals_nav.tasks import geometry
from mhseals_nav.tasks.behaviours import Driver, Leaf
from mhseals_nav.tasks.tracks import Track
import numpy as np
import py_trees
from py_trees.common import Status


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Coverage:
    """Where the camera has looked: last-seen times on a coarse grid over the course."""

    def __init__(self, area=None, cell=1.5, home=(0.0, 0.0, 0.0), radius=60.0,
                 forget_s=120.0):
        if area is None:  # no course box given: a square around where the run started
            area = [[home[0] - radius, home[1] - radius], [home[0] + radius, home[1] + radius]]
        self.lo, self.hi = np.array(area[0], float), np.array(area[1], float)
        self.cell, self.forget_s, self.now = float(cell), float(forget_s), 0.0
        nx, ny = np.maximum(np.ceil((self.hi - self.lo) / self.cell), 1).astype(int)
        self.xs = self.lo[0] + (np.arange(nx) + 0.5) * self.cell
        self.ys = self.lo[1] + (np.arange(ny) + 0.5) * self.cell
        self.seen = np.full((nx, ny), -np.inf)  # -inf: never seen

    @property
    def area(self):
        return [self.lo.tolist(), self.hi.tolist()]

    def contains(self, xy):
        return bool(np.all(self.lo <= xy[:2]) and np.all(np.asarray(xy[:2]) <= self.hi))

    def cone(self, pose, fov):
        """Return the grid window and the mask of its cells in the camera cone at ``pose``.

        Only the cone's bounding box is touched, so a 10 Hz observe stays cheap.
        """
        (hfov, rng), (x, y, yaw) = fov, pose
        arc = yaw + np.linspace(-hfov / 2, hfov / 2, 17)
        box = []
        for c, points, size in ((x, rng * np.cos(arc), len(self.xs)),
                                (y, rng * np.sin(arc), len(self.ys))):
            lo, hi = c + min(points.min(), 0.0), c + max(points.max(), 0.0)
            origin = self.lo[len(box)]
            box.append(slice(int(np.clip((lo - origin) // self.cell - 1, 0, size)),
                             int(np.clip((hi - origin) // self.cell + 2, 0, size))))
        dx = self.xs[box[0], None] - x
        dy = self.ys[None, box[1]] - y
        off = np.abs(np.angle(np.exp(1j * (np.arctan2(dy, dx) - yaw))))
        return tuple(box), (dx * dx + dy * dy <= rng * rng) & (off <= hfov / 2)

    def observe(self, pose, fov, now):
        """Mark the cells in the camera cone as seen at ``now``."""
        window, mask = self.cone(pose, fov)
        self.seen[window][mask] = now
        self.now = now

    def unseen(self, window=(slice(None), slice(None)), now=None):
        """Weight 0..1 per cell: 1 never seen, 0 just seen, back to 1 over ``forget_s``."""
        now = self.now if now is None else now
        return np.clip((now - self.seen[window]) / self.forget_s, 0.0, 1.0)

    def gain(self, pose, fov, now=None):
        """Unseen area (m^2, weighted) the camera would see from ``pose``."""
        window, mask = self.cone(pose, fov)
        return float(self.unseen(window, now)[mask].sum()) * self.cell ** 2

    def bearing_gaps(self, pose, fov, spread, direction, min_gain=1.0, prefer=None,
                     hysteresis=0.3):
        """Return the bearing within +-``spread`` of ``direction`` that sees most unseen.

        Ties go to the bearing nearest ``direction``; ``prefer`` (the last bearing) is
        kept unless another is ``hysteresis`` better. None when all is seen.
        """
        step = max(fov[0] / 4, math.radians(5))
        offsets = sorted(np.arange(-spread, spread + 1e-9, step), key=abs)
        scored = [(self.gain((pose[0], pose[1], wrap(direction + o)), fov), wrap(direction + o))
                  for o in offsets]
        best = max(scored, key=lambda s: s[0])  # max keeps the first (straightest) of ties
        if prefer is not None:
            kept = self.gain((pose[0], pose[1], prefer), fov)
            if kept >= min_gain and kept * (1 + hysteresis) >= best[0]:
                return prefer
        return best[1] if best[0] >= min_gain else None


def view_pose(target_xy, pose, standoff):
    """Return a pose ``standoff`` metres from the target on the boat's side, facing it."""
    back = (geometry.heading(target_xy, pose) if math.dist(target_xy, pose[:2]) > 1e-6
            else pose[2] + math.pi)
    x, y = target_xy[0] + standoff * math.cos(back), target_xy[1] + standoff * math.sin(back)
    return (x, y, wrap(back + math.pi))


def wanted(tracks, labels, cfg):
    """Wanted tracks still worth a look: priors, weak or uncertain live ones."""
    return [t for t in tracks or () if t.label in labels and (
        t.prior or t.confidence < cfg['revisit_confidence']
        or (t.sigma or 0.0) > cfg['revisit_sigma'])]


def view_score(coverage, view, pose, fov, now, targets, cfg):
    """Return (gain, score) of ``view``: unseen m^2 plus track bonuses, over travel cost."""
    gain = coverage.gain(view, fov, now)
    for t in targets:  # tracks in the view's cone: a prior or weak one, and its sigma
        off = abs(wrap(geometry.heading(view, (t.x, t.y)) - view[2]))
        if math.dist(view[:2], (t.x, t.y)) <= fov[1] and off <= fov[0] / 2:
            gain += cfg['track_bonus'] + cfg['sigma_gain'] * (t.sigma or 0.0)
    travel = math.dist(view[:2], pose[:2])
    way = geometry.heading(pose, view) if travel > 1e-6 else pose[2]
    turn = abs(wrap(way - pose[2])) + abs(wrap(view[2] - way))
    return gain, gain / (1.0 + cfg['travel_cost'] * travel + cfg['turn_cost'] * turn)


def next_view(coverage, pose, fov, now, tracks, labels, cfg):
    """Return the best (view pose, score) on rings around the boat, or None under min_gain.

    Candidates face outward and a few headings either side, inside the area.
    """
    targets, best = wanted(tracks, labels, cfg), None
    for radius in cfg['rings']:
        for k in range(cfg['ring_points']):
            out = pose[2] + 2 * math.pi * k / cfg['ring_points']
            xy = (pose[0] + radius * math.cos(out), pose[1] + radius * math.sin(out))
            if not coverage.contains(xy):
                continue
            for h in cfg['headings_deg']:
                view = (*xy, wrap(out + math.radians(h)))
                gain, score = view_score(coverage, view, pose, fov, now, targets, cfg)
                if gain >= cfg['min_gain'] and (best is None or score > best[1]):
                    best = (view, score)
    return best


def lawnmower(area, pose, spacing):
    """Return one route of legs along the area's long side, from the corner nearest the boat."""
    (x0, y0), (x1, y1) = area
    long_x = x1 - x0 >= y1 - y0
    a0, a1, b0, b1 = (x0, x1, y0, y1) if long_x else (y0, y1, x0, x1)
    pa, pb = (pose[0], pose[1]) if long_x else (pose[1], pose[0])
    m = min(spacing / 2, (a1 - a0) / 2, (b1 - b0) / 2)  # stay half a swath inside
    ends = (a0 + m, a1 - m) if abs(pa - a0) <= abs(pa - a1) else (a1 - m, a0 + m)
    count = max(1, int((b1 - b0 - 2 * m) // spacing) + 1)
    rows = np.linspace(b0 + m, b1 - m, count) if count > 1 else [(b0 + b1) / 2]
    rows = rows if abs(pb - b0) <= abs(pb - b1) else rows[::-1]
    route = []
    for i, b in enumerate(rows):
        start, end = ends if i % 2 == 0 else ends[::-1]
        yaw = 0.0 if end > start else math.pi
        for a in (start, end):
            route.append((a, b, yaw) if long_x else (b, a, wrap(math.pi / 2 - yaw)))
    return route


def expanding_square(pose, step, area):
    """Return one route spiralling out square from ``pose`` in ``step`` legs, clipped to area."""
    (x0, y0), (x1, y1) = area
    x, y, yaw = pose
    route, leg = [], 0
    reach = 2 * max(x1 - x0, y1 - y0)
    while (leg // 2 + 1) * step <= reach:
        heading = yaw + leg * math.pi / 2
        length = (leg // 2 + 1) * step
        x = min(max(x + length * math.cos(heading), x0), x1)
        y = min(max(y + length * math.sin(heading), y0), y1)
        if not route or math.dist(route[-1][:2], (x, y)) > 1.0:  # clipped legs collapse
            prev = route[-1] if route else pose
            route.append((x, y, geometry.heading(prev, (x, y))))
        leg += 1
    return route


def found(tracks, labels, cfg):
    """Tell whether the labels are tracked live (``need`` all or any) at ``min_confidence``."""
    live = {t.label for t in tracks or () if not t.prior
            and t.confidence >= cfg['min_confidence']}
    hits = [label in live for label in labels]
    return any(hits) if cfg['need'] == 'any' else all(hits)


def board_tracks(board):
    """Return the ``tracks`` key, else live tracks made from ``objects`` (older interface)."""
    if board.exists('tracks') and board.tracks is not None:
        return board.tracks
    objects = board.objects if board.exists('objects') else None
    return [Track(label, x, y, 1.0, None, True, False, '') for label, x, y in objects or ()]


class SearchLeaf(Driver):
    """A Driver that succeeds as soon as the labels are seen live; plans once per goal."""

    def __init__(self, name, labels, cfg):
        super().__init__(name)
        self.labels, self.cfg, self.own = tuple(labels), cfg, None

    def get(self, key):
        return self.board.get(key) if self.board.exists(key) else None

    def tracks(self):
        return board_tracks(self.board)

    def fov(self):
        fov = self.get('fov')
        return fov or (math.radians(self.cfg['hfov_deg']), self.cfg['range_m'])

    def coverage(self):
        """Return the runner's coverage, else a private one this leaf keeps up itself."""
        shared = self.get('coverage')
        if shared is not None:
            return shared
        if self.own is None:
            home = self.get('home') or self.board.pose
            self.own = Coverage(self.cfg['area'], self.cfg['cell'], home, self.cfg['radius'],
                                self.cfg['forget_s'])
        return self.own

    def send(self, route, why):
        if self.target is None:  # this strategy's first goal: tell the operator why
            self.decide('search', f'{self.labels[0]} not seen: {why}')
        self.target = route[-1]
        self.drive(route, why=why[:60])

    def due(self, state):
        """Plan the next goal: nothing running, or within ``lead`` of the goal's end."""
        return state != 'RUNNING' or (self.target is not None and math.dist(
            self.board.pose[:2], self.target[:2]) < self.cfg['lead'])

    def initialise(self):
        super().initialise()
        self.target = None

    def update(self):
        if found(self.tracks(), self.labels, self.cfg):
            return Status.SUCCESS
        if self.board.pose is None:
            return Status.RUNNING
        if self.get('coverage') is None:  # a private coverage needs this leaf's own looks
            self.coverage().observe(self.board.pose, self.fov(), self.board.now)
        return self.step(self.driving())


class LookAtPriors(SearchLeaf):
    """Visit view poses of prior and weak tracks, nearest first: look where the thing was."""

    def initialise(self):
        super().initialise()
        self.todo = self.current = None

    def confirmed(self, target):
        """Tell whether a confident live track of the label stands where it was looked for."""
        return any(not t.prior and t.label == target.label
                   and t.confidence >= self.cfg['revisit_confidence']
                   and math.dist((t.x, t.y), (target.x, target.y)) < self.cfg['standoff']
                   for t in self.tracks())

    def step(self, state):
        if self.todo is None:
            self.todo = wanted(self.tracks(), self.labels, self.cfg)
        if state == 'RUNNING' and not (self.current and self.confirmed(self.current)):
            return Status.RUNNING
        if not self.todo:
            self.feedback_message = 'no prior or weak track left'
            return Status.FAILURE
        pose = self.board.pose
        t = self.current = min(self.todo, key=lambda t: math.dist((t.x, t.y), pose[:2]))
        self.todo = [o for o in self.todo if math.dist((o.x, o.y), (t.x, t.y)) > 1.0]
        why = f'look at prior {t.label}' if t.prior else f'revisit low-confidence {t.label}'
        self.send([view_pose((t.x, t.y), pose, self.cfg['standoff'])], why)
        return Status.RUNNING


class Scan(SearchLeaf):
    """Drive ``scan_step`` along the least-seen bearing ahead: the camera looks while moving."""

    def initialise(self):
        super().initialise()
        self.scans, self.direction, self.bearing = 0, None, None

    def step(self, state):
        pose = self.board.pose
        if self.direction is None:
            self.direction = pose[2]  # keep to the travel direction: no wandering off
        if not self.due(state):
            return Status.RUNNING
        cover = self.coverage()
        if self.scans >= self.cfg['max_scans']:
            return Status.RUNNING if state == 'RUNNING' else Status.FAILURE
        bearing = cover.bearing_gaps(pose, self.fov(), math.radians(self.cfg['scan_spread_deg']),
                                     self.direction, self.cfg['min_gain'], self.bearing,
                                     self.cfg['hysteresis'])
        if bearing is None:
            self.feedback_message = 'nothing unseen ahead'
            return Status.RUNNING if state == 'RUNNING' else Status.FAILURE
        self.scans, self.bearing = self.scans + 1, bearing
        deg = round(math.degrees(wrap(bearing - self.direction)))
        self.send([geometry.forward((*pose[:2], bearing), self.cfg['scan_step'])],
                  f'look toward unexplored bearing {deg}°')
        return Status.RUNNING


class NextView(SearchLeaf):
    """Repeat next_view, switching mid-goal only for a view ``hysteresis`` better."""

    def initialise(self):
        super().initialise()
        self.views, self.checked = 0, None

    def step(self, state):
        pose, now, cfg = self.board.pose, self.board.now, self.cfg
        recheck = self.checked is None or now - self.checked >= cfg['replan_s']
        if not (self.due(state) or recheck):
            return Status.RUNNING
        self.checked = now
        cover, fov, tracks = self.coverage(), self.fov(), self.tracks()
        best = next_view(cover, pose, fov, now, tracks, self.labels, cfg)
        if state == 'RUNNING' and not self.due(state):
            current = view_score(cover, self.target, pose, fov, now,
                                 wanted(tracks, self.labels, cfg), cfg)
            if current[0] >= cfg['min_gain'] and (
                    best is None or best[1] <= current[1] * (1 + cfg['hysteresis'])):
                return Status.RUNNING  # the current view is still good enough
        if best is None or self.views >= cfg['max_views']:
            self.feedback_message = f'{self.views} views, nothing left worth a look'
            return Status.RUNNING if state == 'RUNNING' else Status.FAILURE
        self.views += 1
        view = best[0]
        deg = round(math.degrees(geometry.heading(pose, view)))
        self.send([view], f'next view {self.views}: bearing {deg}°, score {best[1]:.0f}')
        return Status.RUNNING


class Sweep(SearchLeaf):
    """Drive a lawnmower or expanding square as one route: bounded, guaranteed coverage."""

    def initialise(self):
        super().initialise()
        self.started = False

    def step(self, state):
        if self.started:
            if state == 'RUNNING':
                return Status.RUNNING
            self.feedback_message = f'sweep ended ({state}), nothing found'
            return Status.FAILURE
        self.started, pose, cfg = True, self.board.pose, self.cfg
        area, spacing = self.coverage().area, cfg['spacing_frac'] * self.fov()[1]
        # auto: a known course area is mown; around an unknown one, spiral out from here.
        pattern = cfg['pattern']
        if pattern == 'auto':
            pattern = 'lawnmower' if cfg['area'] else 'expanding_square'
        if pattern == 'lawnmower':
            route = lawnmower(area, pose, spacing)
        else:
            route = expanding_square(pose, spacing, area)
        self.send(route, f'{pattern} sweep, {len(route)} poses')
        return Status.RUNNING


class Found(Leaf):
    """Condition: SUCCESS when the wanted labels are tracked live."""

    def __init__(self, name, labels, cfg):
        super().__init__(name)
        self.labels, self.cfg = tuple(labels), cfg

    def update(self):
        hit = found(board_tracks(self.board), self.labels, self.cfg)
        return Status.SUCCESS if hit else Status.FAILURE


STRATEGIES = (('priors', LookAtPriors), ('scan', Scan), ('next_view', NextView),
              ('sweep', Sweep))


def load_config(overrides=None):
    """Merge config/tasks/search.yaml with a task's own ``search`` overrides."""
    import yaml  # only the runner and tests load config
    path = Path(__file__).resolve().parents[2] / 'config/tasks/search.yaml'
    if not path.exists():  # installed: the package share directory
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config/tasks/search.yaml'
    cfg = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    overrides = overrides or {}
    cfg.update({k: v for k, v in overrides.items() if k != 'enable'})
    cfg['enable'] = {**cfg.get('enable', {}), **overrides.get('enable', {})}
    return cfg


def search(name, labels, cfg):
    """Seen, else the enabled strategies cheapest first; FAILURE once all are exhausted."""
    children = [Found(f'{name} seen', labels, cfg)]
    children += [cls(f'{name} {key}', labels, cfg) for key, cls in STRATEGIES
                 if cfg['enable'].get(key, True)]
    return py_trees.composites.Selector(name, memory=True, children=children)
