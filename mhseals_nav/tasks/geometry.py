"""Pure geometry for task planning: gates, forward points, circles. Poses are (x, y, yaw)."""

import math


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def forward(pose, distance):
    """The point ``distance`` metres ahead of ``pose``, facing the same way."""
    x, y, yaw = pose
    return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)


def gate_midpoints(reds, greens, origin, max_width=8.0):
    """Gates from red/green marker pairs, nearest first.

    Each red pairs with its nearest unused green no wider than ``max_width``.
    A gate faces through its pair, away from ``origin``, so the boat passes
    straight through instead of along the line of markers.
    """
    pairs = sorted((math.dist(r, g), i, j) for i, r in enumerate(reds)
                   for j, g in enumerate(greens) if math.dist(r, g) <= max_width)
    used_red, used_green, gates = set(), set(), []
    for _, i, j in pairs:
        if i in used_red or j in used_green:
            continue
        used_red.add(i)
        used_green.add(j)
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
