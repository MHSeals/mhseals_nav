"""Task recognition on the example course: types, poses, ids, states and the contract."""

import json
import math
import os
import random

from mhseals_nav.tasks.recognition import Recogniser, SCHEMA, STATES
from mhseals_nav.tasks.tracks import Track

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RULES = os.path.join(ROOT, 'config', 'tasks', 'recognition.yaml')
FIXTURE = os.path.join(ROOT, 'test', 'fixtures', 'task_hypotheses.json')
HOME = (0.0, 0.0, 0.0)

# The example course in the map frame, the boat starting at the origin facing +x.
COURSE = [('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6),
          ('red_pole_buoy', 7, 1.5), ('green_pole_buoy', 7, -1.5),
          ('red_pole_buoy', 13, 1.5), ('green_pole_buoy', 13, -1.5),
          ('red_buoy', 19, 1.2), ('green_buoy', 19, -1.2),
          ('red_buoy', 24, -0.8), ('green_buoy', 24, -3.2),
          ('red_buoy', 28.5, -4.3), ('green_buoy', 28.5, -6.7),
          ('red_buoy', 32, -8.8), ('green_buoy', 32, -11.2),
          ('yellow_buoy', 21.5, -0.5), ('yellow_buoy', 30, -7.5),
          ('red_circle', 11, -20), ('green_plus', 12, -20), ('blue_square', 13, -20),
          ('yellow_target_boat', 14, -16),
          ('red_buoy', 3, 12.25), ('green_buoy', 3, 9.75),
          ('black_buoy', 10, 12.6), ('black_buoy', 14, 9.4), ('blue_buoy', 19, 11)]


def tracks(course=COURSE, jitter=0.0, rng=None):
    """Track tuples for ``course``, each moved up to ``jitter`` m."""
    rng = rng or random.Random(1)
    return [Track(label, x + rng.uniform(-jitter, jitter), y + rng.uniform(-jitter, jitter),
                  1.0, 0.2, True, False, str(i)) for i, (label, x, y) in enumerate(course)]


def by_type(hypotheses):
    found = {}
    for h in hypotheses:
        found.setdefault(h['type'], []).append(h)
    return found


def close(pose, x, y, yaw, tol=0.05):
    return (math.dist(pose[:2], (x, y)) < tol
            and abs(math.atan2(math.sin(pose[2] - yaw), math.cos(pose[2] - yaw))) < tol)


def test_full_course_finds_each_task_once_at_its_pose():
    found = by_type(Recogniser.from_yaml(RULES).update(tracks(), HOME, 0.0))
    assert {k: len(v) for k, v in found.items()} == dict.fromkeys(
        ('home_gate', 'nav_channel', 'follow_path', 'speed_gate', 'speed_mark', 'dock'), 1)
    one = {k: v[0] for k, v in found.items()}
    assert close(one['home_gate']['pose'], -2.5, 0, math.pi)
    assert close(one['nav_channel']['pose'], 7, 0, 0)
    assert close(one['follow_path']['pose'], 19, 0, 0)
    assert close(one['speed_gate']['pose'], 3, 11, 0)
    assert close(one['speed_mark']['pose'], 19, 11, 0)
    assert close(one['dock']['pose'], 12, -20, math.pi / 2)
    labels = {k: sorted(s[1] for s in h['support']) for k, h in one.items()}
    assert labels['follow_path'].count('yellow_buoy') == 2
    assert labels['speed_gate'] == ['black_buoy', 'black_buoy', 'green_buoy', 'red_buoy']
    assert 'yellow_target_boat' in labels['dock']
    assert all(h['confidence'] == 1.0 for h in one.values())
    # The channel's box covers both gates.
    xs = [p[0] for p in one['nav_channel']['extent']]
    assert min(xs) < 7 and max(xs) > 13


def test_one_channel_gate_is_half_then_confirmed_with_both():
    rules = Recogniser.from_yaml(RULES).rules
    recogniser = Recogniser(rules)
    first = recogniser.update(tracks(COURSE[2:4]), HOME, 0.0)
    assert len(first) == 1 and first[0]['state'] == 'hypothesis'
    assert abs(first[0]['confidence'] - 0.5) < 1e-6
    both = recogniser.update(tracks(COURSE[2:6]), HOME, rules['confirm_s'])
    assert [(h['id'], h['state'], h['confidence']) for h in both] == [
        (first[0]['id'], 'confirmed', 1.0)]
    # Confirmed holds while support briefly drops out, then goes after forget_s.
    later = rules['confirm_s'] + rules['forget_s']
    assert recogniser.update([], HOME, later)[0]['state'] == 'confirmed'
    assert recogniser.update([], HOME, later + 1.0) == []


def test_ids_are_stable_under_jitter():
    recogniser = Recogniser.from_yaml(RULES)
    ids = {h['id'] for h in recogniser.update(tracks(), HOME, 0.0)}
    rng = random.Random(7)
    for step in range(1, 20):
        again = recogniser.update(tracks(jitter=0.3, rng=rng), HOME, step * 0.5)
        assert {h['id'] for h in again} == ids


def test_done_sticks():
    recogniser = Recogniser.from_yaml(RULES)
    recogniser.update(tracks(), HOME, 0.0)
    recogniser.mark('dock', 'done')
    for now in (1.0, 5.0, 100.0):  # still there long after its support is gone
        dock = [h for h in recogniser.update(tracks() if now < 50 else [], HOME, now)
                if h['type'] == 'dock']
        assert [h['state'] for h in dock] == ['done']


def test_speed_gate_is_not_a_path_gate():
    found = by_type(Recogniser.from_yaml(RULES).update(tracks(), HOME, 0.0))
    path = {(s[2], s[3]) for s in found['follow_path'][0]['support']}
    assert (3, 12.25) not in path and (3, 9.75) not in path
    # A lone small gate with no blue mark is neither a path nor a speed gate.
    lone = Recogniser.from_yaml(RULES).update(tracks(COURSE[20:22]), HOME, 0.0)
    assert lone == []


def test_black_pair_far_from_home_is_not_home_gate():
    far = [('black_buoy', 30, 1.5), ('black_buoy', 30, -1.5)]
    assert Recogniser.from_yaml(RULES).update(tracks(far), HOME, 0.0) == []


def check_message(message):
    """Assert ``message`` follows the mhseals.task_hypotheses/1 contract."""
    assert set(message) == {'schema', 'time', 'frame', 'hypotheses'}
    assert message['schema'] == SCHEMA and message['frame'] == 'map'
    assert isinstance(message['time'], float)
    for h in message['hypotheses']:
        assert set(h) == {'id', 'type', 'state', 'confidence', 'pose', 'extent', 'support'}
        assert h['id'].startswith(h['type'] + '/')
        assert h['type'] in ('nav_channel', 'follow_path', 'speed_gate', 'speed_mark',
                             'dock', 'home_gate')
        assert h['state'] in STATES and 0.0 <= h['confidence'] <= 1.0
        assert len(h['pose']) == 3
        assert len(h['extent']) >= 4 and h['extent'][0] == h['extent'][-1]
        assert all(len(p) == 2 for p in h['extent'])
        assert all(len(s) == 4 and isinstance(s[0], str) and isinstance(s[1], str)
                   for s in h['support'])


def test_message_and_fixture_follow_the_contract():
    recogniser = Recogniser.from_yaml(RULES)
    recogniser.update(tracks(), HOME, 0.0)
    message = recogniser.message(3.0)
    check_message(json.loads(json.dumps(message)))
    with open(FIXTURE, encoding='utf-8') as stream:
        check_message(json.load(stream))
