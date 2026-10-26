"""RViz markers for tracks and lifted observations; built only when someone listens."""

from hashlib import md5
from itertools import product

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

MOTION_TEXT = ('?', 'S', 'M')


# A buoy shows its own colour (red_pole_buoy red, black_cross black); other labels keep a stable hash colour.
PALETTE = {'red': (1.0, 0.16, 0.12), 'green': (0.16, 0.9, 0.3), 'blue': (0.15, 0.4, 1.0),
           'yellow': (1.0, 0.85, 0.15), 'black': (0.25, 0.28, 0.32)}


def rgb_of(label):
    named = PALETTE.get(label.split('_')[0])
    return named or tuple(v / 255.0 for v in md5(label.encode()).digest()[:3])


def colour(label, marker, alpha=1.0):
    marker.color.r, marker.color.g, marker.color.b = rgb_of(label)
    marker.color.a = alpha


def _marker(header, ns, ident, kind, xyz):
    marker = Marker()
    marker.header = header
    marker.ns, marker.id, marker.type, marker.action = ns, ident, kind, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
    marker.pose.orientation.w = 1.0
    return marker


def _deletes(header, namespaces, identifiers):
    out = []
    for ident in identifiers:
        for ns in namespaces:
            marker = Marker(header=header, ns=ns, id=ident, action=Marker.DELETE)
            out.append(marker)
    return out


def track_markers(header, rows, previous, lifetime):
    """rows: (id, class_id, motion, prior, xyz, vxy, size). Returns (MarkerArray, ids)."""
    array, ids = MarkerArray(), set()
    for ident, label, motion, prior, xyz, vxy, size in rows:
        ids.add(ident)
        alpha = 0.5 if prior else 1.0
        body = _marker(header, 'tracked_objects', ident, Marker.SPHERE, xyz)
        body.scale.x = body.scale.y = body.scale.z = max(0.3, *size)
        colour(label, body, alpha)
        text = _marker(header, 'tracked_labels', ident, Marker.TEXT_VIEW_FACING,
                       (xyz[0], xyz[1], xyz[2] + 0.5))
        text.scale.z = 0.3
        text.text = f'{label} {MOTION_TEXT[motion]}{"·" if prior else ""}'
        colour(label, text)
        arrow = _marker(header, 'tracked_velocity', ident, Marker.ARROW, (0, 0, 0))
        arrow.points = [Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                        Point(x=float(xyz[0] + vxy[0]), y=float(xyz[1] + vxy[1]),
                              z=float(xyz[2]))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.12, 0.12
        colour(label, arrow, alpha)
        for marker in (body, text, arrow):
            marker.lifetime.sec = int(lifetime)
            array.markers.append(marker)
    array.markers += _deletes(header, ('tracked_objects', 'tracked_labels', 'tracked_velocity'),
                              sorted(previous - ids))
    return array, ids


def observation_markers(header, rows, previous_count):
    """rows: (class_id, score, xyz, size) in the header frame. Boxes plus a score label."""
    array = MarkerArray()
    for index, (label, score, xyz, size) in enumerate(rows):
        box = _marker(header, 'detection_boxes', index, Marker.LINE_LIST, xyz)
        box.scale.x = 0.02
        colour(label, box)
        corners = list(product(*[(-v / 2, v / 2) for v in (max(s, 0.1) for s in size)]))
        for i, a in enumerate(corners):
            for b in corners[i + 1:]:
                if sum(x != y for x, y in zip(a, b)) == 1:
                    box.points += [Point(x=a[0], y=a[1], z=a[2]), Point(x=b[0], y=b[1], z=b[2])]
        text = _marker(header, 'confidence', index, Marker.TEXT_VIEW_FACING,
                       (xyz[0], xyz[1], xyz[2] + max(size[2], 0.1) / 2 + 0.1))
        text.text, text.scale.z = f'{label} {score:.0%}', 0.15
        colour(label, text)
        for marker in (box, text):
            marker.lifetime.sec = 2
            array.markers.append(marker)
    array.markers += _deletes(header, ('detection_boxes', 'confidence'),
                              range(len(rows), previous_count))
    return array
