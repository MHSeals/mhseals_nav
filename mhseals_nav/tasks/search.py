"""Purposeful search for task elements not seen yet, or seen too uncertainly.

``search(name, labels, cfg)`` tries the cheapest strategy first and stops the moment
the wanted labels are tracked live:
  HeadFor       drive to a view of where the course-layout belief puts the task
  LookAtPriors  look where the course map, or a weak track, says the thing is
  Scan          turn toward the least-seen bearing ahead while moving on
  NextView      next-best-view by information gain over the camera coverage grid
  Sweep         a bounded lawnmower or expanding square: the guaranteed fallback
Goals are re-planned only when one ends or nears its end, or when another view is
clearly (``hysteresis``) better, so the boat turns while moving and never dithers.
"""

import math

from mhseals_nav.tasks import geometry, layout, load_config as load_task_config
from mhseals_nav.tasks.behaviours import Driver, Leaf
from mhseals_nav.tasks.geometry import wrap
from mhseals_nav.tasks.tracks import Track
import numpy as np
import py_trees
from py_trees.common import Status


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

    def gain(self, pose, fov, now=None, weight=None):
        """Unseen area (m^2, weighted) the camera would see from ``pose``.

        ``weight`` (a grid from ``prior``) scales each cell by how likely the target is there.
        """
        window, mask = self.cone(pose, fov)
        unseen = self.unseen(window, now)
        if weight is not None:
            unseen = unseen * weight[window]
        return float(unseen[mask].sum()) * self.cell ** 2

    def prior(self, regions, side=None, floor=0.2):
        """Return a weight grid: ``floor`` everywhere, plus each region's ``weight``.

        A region is {box: [[x0, y0], [x1, y1]], weight, mirror}: a soft placement prior
        from the course rules. ``mirror`` also weights the box mirrored across the start
        line (y -> -y) while the course side is unknown; a known ``side`` (port: y > 0)
        keeps only the box on that side.
        """
        grid = np.full(self.seen.shape, float(floor))
        for region in regions or ():
            (x0, y0), (x1, y1) = region['box']
            boxes = [(y0, y1)]
            if region.get('mirror'):
                boxes.append((-y1, -y0))
                if side is not None:
                    boxes = [b for b in boxes if (b[0] + b[1] > 0) == (side == 'port')]
            inx = (self.xs >= min(x0, x1)) & (self.xs <= max(x0, x1))
            for lo, hi in boxes:
                iny = (self.ys >= min(lo, hi)) & (self.ys <= max(lo, hi))
                grid[np.ix_(inx, iny)] += region.get('weight', 1.0)
        return grid

    def region_area(self, regions, side=None):
        """Return the box around the prior regions on the course's side, inside the area.

        None without regions, or while a mirrored region's side is still unknown.
        """
        boxes = []
        for region in regions or ():
            (x0, y0), (x1, y1) = region['box']
            if region.get('mirror'):
                if side is None:
                    return None
                if side != 'port':
                    y0, y1 = -y1, -y0
            boxes.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
        if not boxes:
            return None
        (ax0, ay0), (ax1, ay1) = self.area
        box = (max(min(b[0] for b in boxes), ax0), max(min(b[1] for b in boxes), ay0),
               min(max(b[2] for b in boxes), ax1), min(max(b[3] for b in boxes), ay1))
        return [box[:2], box[2:]] if box[0] < box[2] and box[1] < box[3] else None

    def sector(self, gate, near, far, spread):
        """Return the mask of cells ``near``-``far`` m past ``gate``, +-``spread`` of its normal.

        ``gate`` is (x, y, normal); ``spread`` in rad.
        """
        dx, dy = self.xs[:, None] - gate[0], self.ys[None, :] - gate[1]
        off = np.abs(np.angle(np.exp(1j * (np.arctan2(dy, dx) - gate[2]))))
        reach = dx * dx + dy * dy
        return (reach >= near * near) & (reach <= far * far) & (off <= spread)

    def region_views(self, regions, pose, rng, side=None):
        """Return a view pose per prior region box: ``rng`` back from its centre, facing it."""
        views = []
        for region in regions or ():
            (x0, y0), (x1, y1) = region['box']
            for sign in ((1, -1) if region.get('mirror') else (1,)):
                centre = ((x0 + x1) / 2, sign * (y0 + y1) / 2)
                if region.get('mirror') and side is not None and (
                        (centre[1] > 0) != (side == 'port')):
                    continue
                if math.dist(centre, pose[:2]) <= rng:
                    view = (pose[0], pose[1], geometry.heading(pose, centre))
                else:
                    view = view_pose(centre, pose, rng)
                if self.contains(view):
                    views.append(view)
        return views

    def bearing_gaps(self, pose, fov, spread, direction, min_gain=1.0, prefer=None,
                     hysteresis=0.3, weight=None):
        """Return the bearing within +-``spread`` of ``direction`` that sees most unseen.

        Ties go to the bearing nearest ``direction``; ``prefer`` (the last bearing) is
        kept unless another is ``hysteresis`` better. None when all is seen.
        """
        step = max(fov[0] / 4, math.radians(5))
        offsets = sorted(np.arange(-spread, spread + 1e-9, step), key=abs)
        scored = [(self.gain((pose[0], pose[1], wrap(direction + o)), fov, weight=weight),
                   wrap(direction + o)) for o in offsets]
        best = max(scored, key=lambda s: s[0])  # max keeps the first (straightest) of ties
        if prefer is not None:
            kept = self.gain((pose[0], pose[1], prefer), fov, weight=weight)
            if kept >= min_gain and kept * (1 + hysteresis) >= best[0]:
                return prefer
        return best[1] if best[0] >= min_gain else None


def view_pose(target_xy, pose, standoff):
    """Return a pose ``standoff`` metres from the target on the boat's side, facing it."""
    back = (geometry.heading(target_xy, pose) if math.dist(target_xy, pose[:2]) > 1e-6
            else pose[2] + math.pi)
    x, y = target_xy[0] + standoff * math.cos(back), target_xy[1] + standoff * math.sin(back)
    return (x, y, wrap(back + math.pi))


def course_side(tracks, cues, min_offset=4.0):
    """Return the course's side S (port: y > 0 in the start frame) from ``cues`` seen, or None.

    Cues are labels only side S carries (the marina's banners and slip lights): the
    mean lateral offset of their tracks beyond ``min_offset`` m tells the side.
    """
    ys = [t.y for t in tracks or () if t.label in cues and not t.prior
          and abs(t.y) >= min_offset]
    if not ys:
        return None
    return 'port' if sum(ys) > 0 else 'starboard'


def cue_gates(tracks, pair, origin, exclude=()):
    """Return the gates (x, y, normal) past which ``pair['label']`` must lie, from tracks.

    With ``gate`` (m) in ``pair``, its two ``with`` labels at most that far apart make a
    gate, and the label lies ``min``-``max`` m past it (the speed turn mark beyond its
    red/green gate): past is away from ``origin``, where the run started, as the course
    runs out from there. Gates within ``exclude_radius`` of an ``exclude`` point (another
    task's cleared gates: red/green look alike) are none.
    """
    if not pair or not pair.get('gate') or len(pair['with']) != 2:
        return []
    side = [[(t.x, t.y) for t in tracks or () if t.label == label] for label in pair['with']]
    gates = geometry.gate_midpoints(side[0], side[1], origin, pair['gate'])
    return geometry.unclaimed(gates, exclude, pair.get('exclude_radius', 0.0))


def wanted(tracks, labels, cfg):
    """Wanted tracks still worth a look: priors, weak or uncertain live ones."""
    return [t for t in tracks or () if t.label in labels and (
        t.prior or t.confidence < cfg['revisit_confidence']
        or (t.sigma or 0.0) > cfg['revisit_sigma'])]


def view_score(coverage, view, pose, fov, now, targets, cfg):
    """Return (gain, score) of ``view``: unseen m^2 plus track bonuses, over travel cost."""
    gain = coverage.gain(view, fov, now, cfg.get('_weight'))
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
    targets, best, views = wanted(tracks, labels, cfg), None, []
    for radius in cfg['rings']:
        for k in range(cfg['ring_points']):
            out = pose[2] + 2 * math.pi * k / cfg['ring_points']
            xy = (pose[0] + radius * math.cos(out), pose[1] + radius * math.sin(out))
            if coverage.contains(xy):
                views += [(*xy, wrap(out + math.radians(h))) for h in cfg['headings_deg']]
    # Placement priors: a view of each likely region, however far (travel is costed).
    views += coverage.region_views(cfg.get('regions'), pose, 0.6 * fov[1], cfg.get('_side'))
    # A gate the target lies past: look on from the gate, and at the stretch past it.
    pair = cfg.get('pair') or {}
    for gate in cfg.get('_cues') or ():
        past = geometry.forward(gate, (pair['min'] + pair['max']) / 2)
        views += [v for v in (gate, view_pose(past[:2], pose, 0.6 * fov[1]))
                  if coverage.contains(v)]
    for view in views:
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
    """Tell whether the labels are tracked live (``need`` all or any) at ``min_confidence``.

    With ``pair`` ({label, with, min, max}), that label counts only beside its companions.
    """
    live = [t for t in tracks or () if not t.prior and t.confidence >= cfg['min_confidence']]
    pair = cfg.get('pair')
    if pair:
        keep = geometry.paired([(t.label, t.x, t.y) for t in live], pair)
        live = [t for t in live if t.label != pair['label'] or (t.label, t.x, t.y) in keep]
    live = {t.label for t in live}
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
        super().__init__(name, reads=('layout', *(cfg.get('pair') or {}).get('exclude', ())))
        self.labels, self.cfg, self.own = tuple(labels), dict(cfg), None

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

    def cues(self):
        """Return the gates the target lies past (``cue_gates``), seen or from the map."""
        pair = self.cfg.get('pair')
        if not pair or not pair.get('gate'):
            return []
        tracks = [t for t in self.tracks()
                  if t.prior or t.confidence >= self.cfg['min_confidence']]
        done = [g for key in pair.get('exclude', ()) for g in self.get(key) or ()]
        gates = cue_gates(tracks, pair, self.get('home') or self.board.pose, done)
        if not self.cfg.get('regions'):
            return gates
        # Only where the course rules put the task: another task's gates are no cue,
        # whether it has been done yet or not (the debris channel before the speed run).
        side = course_side(tracks, self.cfg.get('side_cues') or ())
        return [g for g in gates if layout.inside(g, self.cfg['regions'], side)]

    def prior(self):
        """Put the placement-prior weights, the course side and the cues into the config copy.

        Each cue gate weighs the stretch past it (``cue_weight``, +-``cue_spread_deg``).
        """
        cfg, cues = self.cfg, self.cues()
        if not cfg.get('regions') and not cues and not cfg.get('_cues'):
            return
        side = course_side(self.tracks(), cfg.get('side_cues') or ())
        cover = self.coverage()
        # Cues keyed to 0.5 m, so jittering tracks reuse the weights; a gate's normal is
        # left out: track noise turns it more than the cue spread minds.
        key = (side, [(round(x * 2), round(y * 2)) for x, y, _ in cues])
        if cfg.get('_key') != key or cfg.get('_cover') is not cover:
            cfg['_key'], cfg['_side'], cfg['_cover'], cfg['_cues'] = key, side, cover, cues
            weight = (cover.prior(cfg['regions'], side, cfg['prior_floor'])
                      if cfg.get('regions') else np.ones(cover.seen.shape) if cues else None)
            pair = cfg.get('pair') or {}
            for gate in cues:
                weight[cover.sector(gate, pair['min'], pair['max'],
                                    math.radians(cfg['cue_spread_deg']))] += cfg['cue_weight']
            cfg['_weight'] = weight

    def send(self, route, why):
        if self.target is None:  # this strategy's first goal: tell the operator why
            self.decide('search', f'{self.labels[0]} not seen: {why}')
        self.target, self.arrived = route[-1], None
        self.drive(route, why=why[:60])

    def due(self, state):
        """Plan the next goal: nothing running, or at the goal's end for ``lead_dwell_s``.

        At the end means within ``lead`` m and ``lead_yaw_deg`` of its heading: a goal that
        only turns the boat (a view from where it stands) is not done until it has turned.
        """
        if state != 'RUNNING':
            return True
        pose, goal = self.board.pose, self.target
        if goal is None or math.dist(pose[:2], goal[:2]) >= self.cfg['lead'] or (
                len(goal) > 2 and abs(wrap(pose[2] - goal[2]))
                > math.radians(self.cfg['lead_yaw_deg'])):
            self.arrived = None
            return False
        now = self.get('now') or 0.0
        if self.arrived is None:
            self.arrived = now
        return now - self.arrived >= self.cfg['lead_dwell_s']

    def initialise(self):
        super().initialise()
        self.target = self.arrived = None

    def update(self):
        if found(self.tracks(), self.labels, self.cfg):
            return Status.SUCCESS
        if self.board.pose is None:
            return Status.RUNNING
        if self.get('coverage') is None:  # a private coverage needs this leaf's own looks
            self.coverage().observe(self.board.pose, self.fov(), self.board.now)
        self.prior()
        return self.step(self.driving())


class HeadFor(SearchLeaf):
    """Drive to a view of where the course-layout belief puts the search's ``task``.

    A seen cue gate (``pair``) narrows it to the stretch past the gate. The view stands ``head_standoff`` of the camera range short of the guess and faces
    it, so the camera is on the likely place all the way in; the goal moves only when
    the guess moves ``head_replan`` m (what the camera has passed counts against it).
    Done (FAILURE: on to the other strategies) on arrival, or with no guess.
    """

    def initialise(self):
        super().initialise()
        self.checked = None

    def step(self, state):
        board, cfg = self.board, self.cfg
        recheck = self.checked is None or board.now - self.checked >= cfg['replan_s']
        if self.target is not None and state != 'RUNNING':
            return Status.FAILURE  # there: the other strategies take it from here
        if not recheck:
            return Status.RUNNING
        self.checked = board.now
        belief, task = self.get('layout'), cfg.get('task')
        tracks = self.tracks()
        side = course_side(tracks, cfg.get('side_cues') or ())
        guess = belief.guess(task, self.coverage(), tracks, side,
                             cfg['min_confidence']) if belief and task else None
        pair, cues = cfg.get('pair') or {}, cfg.get('_cues') or ()
        if cues:  # the target lies past a seen gate: the middle of that stretch
            gate = min(cues, key=lambda g: math.dist(g[:2], board.pose[:2]))
            spot, why = geometry.forward(gate, (pair['min'] + pair['max']) / 2)[:2], 'past the gate'
        elif guess is not None and not guess.found:
            spot, why = guess.xy, f'the likely {task} ({guess.spread:.0f} m spread)'
        else:
            return Status.RUNNING if state == 'RUNNING' else Status.FAILURE
        standoff = cfg['head_standoff'] * self.fov()[1]
        view = view_pose(spot, board.pose, standoff)
        if self.target is None and math.dist(spot, board.pose[:2]) <= standoff + cfg['lead']:
            return Status.FAILURE  # already where the spot is best seen from
        if self.target is None or math.dist(view[:2], self.target[:2]) > cfg['head_replan']:
            self.send([view], f'head for {why}')
        return Status.RUNNING


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
                                     self.cfg['hysteresis'], self.cfg.get('_weight'))
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
        # Where the rules put the target, once the course side is known: mow that first.
        self.prior()
        cover, spacing = self.coverage(), cfg['spacing_frac'] * self.fov()[1]
        area = cover.region_area(cfg.get('regions'), cfg.get('_side')) or cover.area
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


STRATEGIES = (('head_for', HeadFor), ('priors', LookAtPriors), ('scan', Scan), ('next_view', NextView),
              ('sweep', Sweep))


def load_config(overrides=None):
    """Merge config/tasks/search.yaml with a task's own ``search`` overrides."""
    overrides = overrides or {}
    cfg = load_task_config('search', {k: v for k, v in overrides.items() if k != 'enable'})
    cfg['enable'] = {**cfg.get('enable', {}), **overrides.get('enable', {})}
    return cfg


def search(name, labels, cfg):
    """Seen, else the enabled strategies cheapest first; FAILURE once all are exhausted."""
    children = [Found(f'{name} seen', labels, cfg)]
    children += [cls(f'{name} {key}', labels, cfg) for key, cls in STRATEGIES
                 if cfg['enable'].get(key, True)]
    return py_trees.composites.Selector(name, memory=True, children=children)
