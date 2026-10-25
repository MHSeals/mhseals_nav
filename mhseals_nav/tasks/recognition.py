"""
Task recognition: which RoboBoat task stands where, inferred from tracked objects.

``Recogniser`` turns ``tracks.Track`` lists into task hypotheses, the
``mhseals.task_hypotheses/1`` message. Each hypothesis has a stable id, a pose (the
task's entrance, facing the way in), a closed extent polygon and the tracks that
support it. Rules and every number come from config/tasks/recognition.yaml.
Pure Python: no ROS imports, so it is testable on any host.
"""

import math

from mhseals_nav.tasks import geometry
import yaml

SCHEMA = 'mhseals.task_hypotheses/1'
STATES = ('hypothesis', 'confirmed', 'done', 'failed')


def _box(points, yaw, margin):
    """Return a closed CCW rectangle along ``yaw``, ``margin`` around ``points``."""
    c, s = math.cos(yaw), math.sin(yaw)
    along = [x * c + y * s for x, y in points]
    across = [-x * s + y * c for x, y in points]
    a0, a1 = min(along) - margin, max(along) + margin
    b0, b1 = min(across) - margin, max(across) + margin
    corners = [(a0, b0), (a1, b0), (a1, b1), (a0, b1), (a0, b0)]
    return [(a * c - b * s, a * s + b * c) for a, b in corners]


def _hull(points):
    """Counter-clockwise convex hull (Andrew's monotone chain), closed."""
    points = sorted(set(points))
    if len(points) < 3:
        return points + points[:1]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1] + lower[:1]


def _inside(point, hull, margin):
    """Tell whether ``point`` is inside the closed CCW ``hull`` or within ``margin`` of it."""
    for a, b in zip(hull, hull[1:]):
        edge = math.dist(a, b) or 1.0
        left = ((b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])) / edge
        if left < -margin:
            return False
    return len(hull) > 3


def _xy(tracks):
    return [(t.x, t.y) for t in tracks]


class Recogniser:
    """Keeps task hypotheses across updates, with stable ids and sticky outcomes."""

    def __init__(self, rules):
        """Take the rules dict of config/tasks/recognition.yaml."""
        self.rules = rules
        self._hypotheses = {}  # id -> internal record, insertion ordered
        self._counts = {}      # type -> next id number
        self._marks = {}       # type -> done | failed, set by the mission runner

    @classmethod
    def from_yaml(cls, path):
        """Build a recogniser from the rules in the YAML file at ``path``."""
        with open(path, encoding='utf-8') as stream:
            return cls(yaml.safe_load(stream))

    def mark(self, kind, state):
        """Set every hypothesis of type ``kind`` to ``state``; done and failed stick."""
        if state not in STATES:
            raise ValueError(f'unknown state {state!r}')
        if state in ('done', 'failed'):
            self._marks[kind] = state
        else:
            self._marks.pop(kind, None)
        for h in self._hypotheses.values():
            if h['type'] == kind:
                h['state'] = state

    def update(self, tracks, home, now):
        """Recognise tasks among ``tracks`` (home is the start pose); return the hypotheses."""
        r = self.rules
        pool = [t for t in tracks if t.confidence >= r['min_confidence']
                and (r['use_priors'] or not t.prior)]
        used, found = set(), []
        for kind in r['priority']:  # earlier types claim their tracks first
            free = [t for t in pool if id(t) not in used]
            for candidate in getattr(self, '_' + kind)(free, home):
                used.update(id(t) for t in candidate['core'] + candidate['extra'])
                found.append(candidate)
        missing = dict(self._hypotheses)
        for candidate in found:
            self._merge(candidate, missing, now)
        for key, h in missing.items():  # unsupported this update
            if h['state'] in ('hypothesis', 'confirmed') and now - h['seen'] > r['forget_s']:
                del self._hypotheses[key]
        return self.hypotheses()

    def hypotheses(self):
        """Return the current hypotheses as contract dicts."""
        return [{key: h[key] for key in
                 ('id', 'type', 'state', 'confidence', 'pose', 'extent', 'support')}
                for h in self._hypotheses.values()]

    def message(self, now):
        """Return the full ``mhseals.task_hypotheses/1`` message."""
        return {'schema': SCHEMA, 'time': float(now), 'frame': 'map',
                'hypotheses': self.hypotheses()}

    def _merge(self, candidate, missing, now):
        """Match a candidate to the nearest same-type hypothesis, or start a new one."""
        kind, pose = candidate['type'], candidate['pose']
        near = [h for h in missing.values() if h['type'] == kind
                and math.dist(h['pose'][:2], pose[:2]) <= self.rules['match_radius']]
        h = min(near, key=lambda h: math.dist(h['pose'][:2], pose[:2]), default=None)
        if h is None:
            number = self._counts.get(kind, 0)
            self._counts[kind] = number + 1
            h = {'id': f'{kind}/{number}', 'type': kind, 'since': now,
                 'state': self._marks.get(kind, 'hypothesis')}
            self._hypotheses[h['id']] = h
        else:
            del missing[h['id']]
        core = candidate['core']
        confidence = sum(t.confidence for t in core) / len(core) * candidate['completeness']
        h.update(seen=now, confidence=round(min(confidence, 1.0), 3),
                 pose=[round(v, 3) for v in pose],
                 extent=[[round(x, 3), round(y, 3)] for x, y in candidate['extent']],
                 support=[[t.id, t.label, round(t.x, 3), round(t.y, 3)]
                          for t in core + candidate['extra']])
        if (h['state'] == 'hypothesis' and confidence >= self.rules['confirm']
                and now - h['since'] >= self.rules['confirm_s']):
            h['state'] = 'confirmed'

    @staticmethod
    def _gates(a, b, home, rule):
        """Gates as (pose, a_track, b_track) from two track lists (one list: same colour)."""
        same = a is b
        pa = _xy(a)
        pb = pa if same else _xy(b)
        gates = []
        for gate in geometry.gate_midpoints(pa, pb, home, rule['max_width'], rule['min_width']):
            # gate_midpoints gives poses only; find the pair whose midpoint it is.
            gates.append(next((gate, s, t) for s in a for t in b if s is not t and math.dist(
                ((s.x + t.x) / 2, (s.y + t.y) / 2), gate[:2]) < 1e-6))
        return gates

    def _of(self, tracks, label):
        return [t for t in tracks if t.label == label]

    def _candidate(self, kind, pose, extent, core, completeness, extra=()):
        return {'type': kind, 'pose': pose, 'extent': extent, 'core': list(core),
                'extra': list(extra), 'completeness': min(completeness, 1.0)}

    def _home_gate(self, tracks, home):
        rule = self.rules['home_gate']
        blacks = self._of(tracks, rule['marker'])
        for gate, s, t in self._gates(blacks, blacks, home, rule):
            if math.dist(gate[:2], home[:2]) <= rule['home_radius']:
                return [self._candidate('home_gate', gate, _box(_xy([s, t]), gate[2],
                                                                rule['margin']), [s, t], 1)]
        return []

    def _nav_channel(self, tracks, home):
        rule = self.rules['nav_channel']
        gates = self._gates(self._of(tracks, rule['red']), self._of(tracks, rule['green']),
                            home, rule)
        if not gates:
            return []
        entrance = gates[0]  # nearest home
        x, y, yaw = entrance[0]
        chosen = [entrance]
        for other in gates[1:]:
            ox, oy, oyaw = other[0]
            spacing = abs((ox - x) * math.cos(yaw) + (oy - y) * math.sin(yaw))
            if (abs(math.cos(oyaw - yaw)) >= math.cos(math.radians(rule['align_deg']))
                    and rule['min_spacing'] <= spacing <= rule['max_spacing']):
                chosen.append(other)
                break
        core = [t for _, s, g in chosen for t in (s, g)]
        return [self._candidate('nav_channel', entrance[0],
                                _box(_xy(core), yaw, rule['margin']), core,
                                len(chosen) / rule['gates'])]

    def _follow_path(self, tracks, home):
        rule = self.rules['follow_path']
        chains = []  # gates nearest home first, each joins the first chain it links to
        for gate in self._gates(self._of(tracks, rule['red']), self._of(tracks, rule['green']),
                                home, rule):
            link = next((c for c in chains if any(
                rule['link_min'] <= math.dist(gate[0][:2], g[0][:2]) <= rule['link_max']
                for g in c)), None)
            if link:
                link.append(gate)
            else:
                chains.append([gate])
        found, species = [], self._of(tracks, rule['species'])
        for chain in (c for c in chains if len(c) >= 2):
            core = [t for _, s, g in chain for t in (s, g)]
            hull = _hull(_xy(core))
            extra = [t for t in species if _inside((t.x, t.y), hull, rule['margin'])]
            species = [t for t in species if all(t is not e for e in extra)]
            found.append(self._candidate('follow_path', chain[0][0], _hull(_xy(core + extra)),
                                         core, len(chain) / rule['gates'], extra))
        return found

    def _speed_gate(self, tracks, home):
        rule = self.rules['speed_gate']
        marks, found = self._of(tracks, rule['mark']), []
        for (x, y, yaw), s, t in self._gates(self._of(tracks, rule['red']),
                                             self._of(tracks, rule['green']), home, rule):
            c, n = math.cos(yaw), math.sin(yaw)
            # The nearest blue along the gate normal, on either side, near its axis.
            ahead = [((m.x - x) * c + (m.y - y) * n, m) for m in marks
                     if abs(-(m.x - x) * n + (m.y - y) * c) <= rule['mark_lateral']]
            ahead = [(a, m) for a, m in ahead if rule['mark_min'] <= abs(a) <= rule['mark_max']]
            if not ahead:
                continue
            along, mark = min(ahead, key=lambda am: abs(am[0]))
            marks.remove(mark)
            way = yaw if along > 0 else math.atan2(-n, -c)
            corridor = _box(_xy([s, t, mark]), way, rule['margin'])
            spills = [b for b in self._of(tracks, rule['spill'])
                      if _inside((b.x, b.y), corridor, 0)]
            found.append(self._candidate('speed_gate', (x, y, way), corridor, [s, t], 1, spills))
            found.append(self._candidate('speed_mark', (mark.x, mark.y, way),
                                         _box([(mark.x, mark.y)], way, rule['margin']),
                                         [mark], 1))
        return found

    def _dock(self, tracks, home):
        rule = self.rules['dock']
        groups = []  # banners linked by the bay spacing
        for t in (t for t in tracks if t.label in rule['banners']):
            link = next((g for g in groups if any(
                rule['link_min'] <= math.dist((t.x, t.y), (b.x, b.y)) <= rule['link_max']
                for b in g)), None)
            if link:
                link.append(t)
            else:
                groups.append([t])
        found = []
        for group in (g for g in groups if len(g) >= 2):
            points = _xy(group)
            centre = (sum(p[0] for p in points) / len(points),
                      sum(p[1] for p in points) / len(points))
            opening = geometry.dock_axis(points, centre, home)
            c, s = math.cos(opening), math.sin(opening)
            if any(abs((p[0] - centre[0]) * c + (p[1] - centre[1]) * s) > rule['collinear']
                   for p in points):
                continue  # not a straight spine
            bays = points + [(p[0] + rule['depth'] * c, p[1] + rule['depth'] * s)
                             for p in points]
            vessels = [t for t in tracks if t.label in rule['vessels']
                       and math.dist((t.x, t.y), centre) <= rule['vessel_radius']]
            found.append(self._candidate('dock', (*centre, opening),
                                         _box(bays, opening, rule['margin']), group,
                                         len(group) / rule['count'], vessels))
        return found
