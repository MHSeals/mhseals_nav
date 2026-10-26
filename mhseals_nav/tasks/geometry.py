"""Pure geometry for task planning: gates, forward points, circles. Poses are (x, y, yaw)."""

import math


def wrap(angle):
    """``angle`` in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def forward(pose, distance):
    """Return the point ``distance`` metres ahead of ``pose``, facing the same way."""
    x, y, yaw = pose
    return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)


def gate_midpoints(reds, greens, origin, max_width=8.0, min_width=1.0, red_side='any'):
    """Gates from red/green marker pairs, nearest first.

    Each red pairs with its nearest unused green between ``min_width`` and
    ``max_width`` apart. A gate faces through its pair: the way that keeps red
    on ``red_side`` (port or starboard), so the colour order sets the way in;
    with ``any``, away from ``origin``. The boat passes straight through instead
    of along the line of markers. Passing the same list twice pairs markers of
    one colour (the black return gate); each marker is then used once.
    """
    same = reds is greens
    pairs = sorted((math.dist(r, g), i, j) for i, r in enumerate(reds)
                   for j, g in enumerate(greens)
                   if min_width <= math.dist(r, g) <= max_width and not (same and j <= i))
    used_red, used_green, gates = set(), set(), []
    for _, i, j in pairs:
        if i in used_red or j in used_green:
            continue
        used_red.add(i)
        used_green.add(j)
        if same:
            used_red.add(j)
            used_green.add(i)
        red, green = reds[i], greens[j]
        middle = ((red[0] + green[0]) / 2, (red[1] + green[1]) / 2)
        # The two normals of the red-green line: this one keeps red to port.
        normal = heading(red, green) + math.pi / 2
        if red_side == 'starboard' or (
                red_side == 'any' and math.cos(normal - heading(origin, middle)) < 0):
            normal += math.pi
        gates.append((*middle, wrap(normal)))
    return sorted(gates, key=lambda gate: math.dist(origin[:2], gate[:2]))


def circle(center, radius, points, clockwise, start):
    """``points`` poses around ``center``, beginning on the side nearest ``start``.

    Each pose faces along the circle, so Nav2 drives around rather than at it.
    """
    begin = heading(center, start)
    sign = -1 if clockwise else 1
    poses = []
    for index in range(1, points + 1):
        angle = begin + sign * 2 * math.pi * index / points
        tangent = angle + sign * math.pi / 2
        poses.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle),
                      wrap(tangent)))
    return poses


def reverse(route):
    """Return the same waypoints in reverse, each turned around."""
    return [(x, y, wrap(yaw + math.pi)) for x, y, yaw in reversed(route)]


def relative(pose, offsets):
    """Waypoints given as (forward, left) metres from ``pose``, facing along the route."""
    x, y, yaw = pose
    cos, sin = math.cos(yaw), math.sin(yaw)
    points = [(x + ahead * cos - left * sin, y + ahead * sin + left * cos)
              for ahead, left in offsets]
    route, previous = [], (x, y)
    for point in points:
        route.append((*point, heading(previous, point)))
        previous = point
    return route


def through(gate, before=1.5, after=1.1):
    """Two poses that carry the whole hull across a gate's line, not just to it.

    ``after`` is half the hull (0.53 m) and a margin: far enough to clear the
    line, near enough not to land on what lies beyond the gate.
    """
    x, y, yaw = gate
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x - before * c, y - before * s, yaw), (x + after * c, y + after * s, yaw)]


def chain(gates, corner_side, after):
    """One route through ``gates`` in order, with a mid-point where the route turns.

    When the next gate's lead-in lies more than ``corner_side`` m to the side of the
    last gate's line, the path turns between them; a pose half way between the two,
    facing along that leg, keeps the controller from cutting the corner over a buoy.
    Each exit lies ``after`` m past its gate (see ``through``).
    """
    route = []
    for gate in gates:
        lead_in, out = through(gate, after=after)
        if route:
            last = route[-1]
            yaw = last[2]
            side = -(lead_in[0] - last[0]) * math.sin(yaw) + (lead_in[1] - last[1]) * math.cos(yaw)
            if abs(side) > corner_side:
                leg = heading(last, lead_in)
                # The exit already faces the leg, so the turn begins at the gate.
                route[-1] = (last[0], last[1], leg)
                route.append(((last[0] + lead_in[0]) / 2, (last[1] + lead_in[1]) / 2, leg))
        route += [lead_in, out]
    return route


def next_gate(gates, pose, cleared, direction=None, horizon=25.0, separation=2.0):
    """Return the nearest gate ahead that has not been cleared, or None.

    Ahead is along ``direction`` (the last cleared gate's heading, so a wobble
    in the boat's own heading does not count gates behind it), else the boat's
    heading. Gates within ``separation`` of a cleared one are the same gate.
    """
    yaw = pose[2] if direction is None else direction
    ahead = [gate for gate in gates
             if (gate[0] - pose[0]) * math.cos(yaw) + (gate[1] - pose[1]) * math.sin(yaw) > 0
             and math.dist(gate[:2], pose[:2]) <= horizon
             and all(math.dist(gate[:2], done[:2]) > separation for done in cleared)]
    return min(ahead, key=lambda gate: math.dist(gate[:2], pose[:2]), default=None)


def unique(points, separation=1.5):
    """Points merged when closer than ``separation``: one per physical object."""
    kept = []
    for point in points:
        if all(math.dist(point, other) >= separation for other in kept):
            kept.append(point)
    return kept


def unclaimed(gates, done, radius):
    """``gates`` farther than ``radius`` from every ``done`` gate: those not another task's.

    Red/green gates look alike, so a gate another task already passed is not this one's.
    """
    return [g for g in gates if all(math.dist(g[:2], d[:2]) > radius for d in done)]


def segment_distance(point, a, b):
    """Distance from ``point`` to the segment from ``a`` to ``b``."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = dx * dx + dy * dy
    t = 0.0 if length == 0 else max(0.0, min(1.0, (
        (point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length))
    return math.dist(point, (a[0] + t * dx, a[1] + t * dy))


def paired(objects, pair):
    """Return the (label, x, y) objects of ``pair['label']`` with each of ``pair['with']`` near.

    Near is between ``pair['min']`` and ``pair['max']`` m. It tells a task's mark from
    a look-alike: the speed turn mark lies 12-31 m past its red/green gate, the harbor
    buoy off the marina has no such gate.
    """
    return [(label, x, y) for label, x, y in objects if label == pair['label'] and all(
        any(other == w and pair['min'] <= math.dist((ox, oy), (x, y)) <= pair['max']
            for other, ox, oy in objects) for w in pair['with'])]


def spin(pose, turns, step=math.pi / 2):
    """Poses turning in place ``turns`` full circles, a quarter turn per pose."""
    x, y, yaw = pose
    count = round(turns * 2 * math.pi / step)
    return [(x, y, wrap(yaw + k * step)) for k in range(1, count + 1)]


def dock_axis(banners, banner, pose):
    """Direction a dock's bays open toward, at ``banner``, toward ``pose`` (a point).

    Banners stand along the dock's spine, so with two or more the spine is the
    line through them and the opening is its normal on the side of ``pose``. With
    one, the opening is taken to face ``pose``. ``pose`` is the boat, unless the
    caller knows better where the open side lies (``opening_side``).
    """
    others = [b for b in banners if math.dist(b, banner) > 0.3]
    toward_boat = heading(banner, pose)
    if not others:
        return toward_boat
    points = [banner, *others]
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    spine = 0.5 * math.atan2(2 * sxy, sxx - syy)  # principal direction of the banners
    normal = spine + math.pi / 2
    if math.cos(normal - toward_boat) < 0:
        normal += math.pi
    return wrap(normal)


def opening_side(pier, banners, vessels, pose, reach=3.0):
    """Return a point on the side a pier's slips open toward.

    The slips of a marina's piers open toward each other (2026 rules), so with
    another pier's banners (any in ``banners`` not on ``pier``) it is their middle;
    else a vessel moored within ``reach`` of the pier, which lies in a slip; else
    the boat (``pose``), which may stand behind the pier.
    """
    other = [b for b in banners if b not in pier]
    moored = [v for v in vessels if any(math.dist(v, b) <= reach for b in pier)]
    for points in (other, moored):
        if points:
            return (sum(p[0] for p in points) / len(points),
                    sum(p[1] for p in points) / len(points))
    return pose[:2]


def bay(banner, opening, berth=0.85, staging=3.0, depth=1.5):
    """Poses for one bay: staging out front, berth inside facing the spine, and its centre.

    ``berth`` is the boat centre's distance from the banner (half a hull plus a
    margin), ``depth`` how far the bay reaches out from the spine.
    """
    c, s = math.cos(opening), math.sin(opening)
    facing = math.atan2(-s, -c)
    return {'staging': (banner[0] + staging * c, banner[1] + staging * s, facing),
            'berth': (banner[0] + berth * c, banner[1] + berth * s, facing),
            'centre': (banner[0] + depth / 2 * c, banner[1] + depth / 2 * s)}


def along(gate, pose):
    """How far ``pose`` is past ``gate``'s line, along the way through it (m)."""
    return (pose[0] - gate[0]) * math.cos(gate[2]) + (pose[1] - gate[1]) * math.sin(gate[2])


def lined_up(gate, pose, before=1.5, half_width=1.0):
    """Tell whether ``pose`` is between the gate's lead-in and its line, near the middle."""
    g = math.cos(gate[2]), math.sin(gate[2])
    dx, dy = pose[0] - gate[0], pose[1] - gate[1]
    along, across = dx * g[0] + dy * g[1], -dx * g[1] + dy * g[0]
    return -before + 0.5 < along < 0 and abs(across) <= half_width


def crossed(gate, pose, previous=None, beyond=0.6, half_width=2.5):
    """Tell whether the boat crossed ``gate``: past its line, within reach of its middle.

    With ``previous`` (the pose a tick before), a crossing between the two counts
    too, so a fast boat or a slow tick cannot skip over the check.
    """
    x, y, yaw = gate
    c, s = math.cos(yaw), math.sin(yaw)

    def frame(p):
        return (p[0] - x) * c + (p[1] - y) * s, -(p[0] - x) * s + (p[1] - y) * c

    along, across = frame(pose)
    if along < beyond:
        return False
    if abs(across) <= half_width:
        return True
    if previous is None:
        return False
    before, side = frame(previous)
    if before >= 0:
        return False
    at = side + (across - side) * (0 - before) / (along - before)  # where it met the line
    return abs(at) <= half_width


def look_around(pose, angle=1.05):
    """Return a forward camera's sweep: turn in place to one side, then to the other.

    Two single-pose routes, sent one after the other: Nav2 drops waypoints within
    a metre of the boat and calls a route done when the boat already sits at its
    end, so a sweep through several poses back to the start would never turn.
    """
    x, y, yaw = pose
    return [[(x, y, wrap(yaw + angle))], [(x, y, wrap(yaw - angle))]]
