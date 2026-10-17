"""Pure geometry for task planning: gates, forward points, circles. Poses are (x, y, yaw)."""

import math


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def forward(pose, distance):
    """The point ``distance`` metres ahead of ``pose``, facing the same way."""
    x, y, yaw = pose
    return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)


def gate_midpoints(reds, greens, origin, max_width=8.0, min_width=1.0):
    """Gates from red/green marker pairs, nearest first.

    Each red pairs with its nearest unused green between ``min_width`` and
    ``max_width`` apart. A gate faces through its pair, away from ``origin``,
    so the boat passes straight through instead of along the line of markers.
    Passing the same list twice pairs markers of one colour (the black
    return gate); each marker is then used once.
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
        # The two normals of the red-green line; take the one pointing away from origin.
        normal = heading(red, green) + math.pi / 2
        away = heading(origin, middle)
        if math.cos(normal - away) < 0:
            normal += math.pi
        gates.append((*middle, math.atan2(math.sin(normal), math.cos(normal))))
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
                      math.atan2(math.sin(tangent), math.cos(tangent))))
    return poses


def reverse(route):
    """The same waypoints in reverse, each turned around."""
    return [(x, y, math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi)))
            for x, y, yaw in reversed(route)]


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


def through(gate, before=1.5, after=2.0):
    """Two poses that carry the whole hull across a gate's line, not just to it."""
    x, y, yaw = gate
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x - before * c, y - before * s, yaw), (x + after * c, y + after * s, yaw)]


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


def spin(pose, turns, step=math.pi / 2):
    """Poses turning in place ``turns`` full circles, a quarter turn per pose."""
    x, y, yaw = pose
    count = round(turns * 2 * math.pi / step)
    return [(x, y, math.atan2(math.sin(yaw + k * step), math.cos(yaw + k * step)))
            for k in range(1, count + 1)]


def dock_axis(banners, banner, pose):
    """Direction a dock's bays open toward, at ``banner``, seen from ``pose``.

    Banners stand along the dock's spine, so with two or more the spine is the
    line through them and the opening is its normal on the boat's side. With
    one, the opening is taken to face the boat (the camera can only see a
    banner from the open side).
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
    return math.atan2(math.sin(normal), math.cos(normal))


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


def remember(memory, seen, now, horizon=5.0, separation=1.0):
    """Merge the latest tracked objects into ``memory`` and forget stale ones.

    ``memory`` and the result are (label, x, y, last_seen); ``seen`` is
    (label, x, y). A sighting within ``separation`` of a remembered object of
    the same label updates it, so a track that drops out for a moment stays.
    """
    kept = [item for item in memory if now - item[3] <= horizon]
    for label, x, y in seen:
        match = next((i for i, item in enumerate(kept)
                      if item[0] == label and math.dist((x, y), item[1:3]) < separation), None)
        if match is None:
            kept.append((label, x, y, now))
        else:
            kept[match] = (label, x, y, now)
    return kept


def crossed(gate, pose, beyond=0.6, half_width=2.5):
    """Has the boat at ``pose`` crossed ``gate``: past its line, within reach of its middle?"""
    x, y, yaw = gate
    dx, dy = pose[0] - x, pose[1] - y
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    across = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return along >= beyond and abs(across) <= half_width
