"""The whole mission on crane_ml's roboboat_2025 layout, in a kinematic world.

The boat drives each Nav2 route at a steady speed, Nav2-style (via-poses within a
metre are passed), and the camera sees only what is inside its cone and range, as
the sim's would. It checks the task logic end to end: every stage in succession,
search when things are out of view, and no stop-and-go between gates.
"""

from fnmatch import fnmatch
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from mhseals_nav.navigation_config import load_strict
from mhseals_nav.tasks.host import config_path, load_params
from mhseals_nav.tasks.tracks import Track
import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('py_trees') is None,
                                reason='needs py_trees')
if importlib.util.find_spec('py_trees') is not None:
    from mhseals_nav.tasks.host import MissionHost
    import py_trees


# crane_ml Tools/RoboBoat/courses/roboboat_2025.json in the start frame; the dock's
# banners stand on its spine behind each bay (course truth bays at y -18.25).
COURSE = [
    ('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6),
    ('red_pole_buoy', 7.0, 1.5), ('green_pole_buoy', 7.0, -1.5),
    ('red_pole_buoy', 13.0, 1.5), ('green_pole_buoy', 13.0, -1.5),
    ('red_buoy', 19.0, 1.2), ('green_buoy', 19.0, -1.2), ('yellow_buoy', 21.5, 0.5),
    ('red_buoy', 24.0, -0.8), ('green_buoy', 24.0, -3.2), ('yellow_buoy', 26.5, -2.6),
    ('black_target_boat', 27.0, 2.5),
    ('red_buoy', 28.5, -4.3), ('green_buoy', 28.5, -6.7), ('yellow_buoy', 30.5, -6.0),
    ('red_buoy', 32.0, -8.8), ('green_buoy', 32.0, -11.2),
    ('blue_square', 10.0, -20.0), ('red_circle', 12.0, -20.0), ('blue_square', 14.0, -20.0),
    ('black_target_boat', 10.0, -18.25),
    ('red_buoy', 3.0, 12.25), ('green_buoy', 3.0, 9.75),
    ('black_buoy', 10.0, 12.6), ('black_buoy', 14.0, 9.4), ('blue_buoy', 19.0, 11.0),
]


BASE = ('schema', 'task', 'event', 'time', 'elapsed')  # every event's own keys


class Goal:
    def __init__(self, route, precise):
        self.route, self.precise = [tuple(p) for p in route], precise
        self.done, self.status = False, None

    def cancel(self):
        if not self.done:
            self.done, self.status = True, 'CANCELED'


TRACKER = load_strict((Path(__file__).resolve().parents[1] / 'config/tracker.yaml').read_text(
    encoding='utf-8'), 'tracker.yaml')['/**']['ros__parameters']


def hold_over(label):
    """Return how long (s) the object tracker keeps a track of ``label`` after its last sighting.

    Its timeout for the class group's prior motion: a buoy is STATIC (900 s); a class
    of no group (banners, light beacons) stays UNKNOWN when seen from beyond ~9 m, so it
    goes 10 s after it leaves the view (test_a_marker_seen_from_afar_stays_unknown_and_goes_soon in
    test_track_filter); a boat 3 s.
    """
    groups = TRACKER['class_groups']
    group = next((g for g, spec in groups.items() if g != 'unknown' and any(
        fnmatch(label, p) for p in spec['match'])), 'unknown')
    return TRACKER['timeout'][groups[group]['prior']]


class Course:
    """A boat that drives Nav2 routes at ``speed`` and a camera with a cone of view.

    The tracks it hands the host are the object tracker's: everything seen, until the
    tracker's hold-over for its class runs out (``hold_over``).
    """

    def __init__(self, params, dt=0.5, speed=1.0, hfov=math.radians(110), range_m=20.0,
                 course=COURSE, host=True):
        self.dt, self.speed, self.fov, self.course = dt, speed, (hfov, range_m), course
        py_trees.blackboard.Blackboard.clear()  # one run per process, as for real
        self.pose, self.seen, self.goal, self.now = (0.0, 0.0, 0.0), {}, None, 0.0
        self.events, self.goals, self.stops, self.path = [], [], 0, 0.0
        if host:  # else the caller drives its own host (the ROS node, in its test)
            self.host = MissionHost('mission', params, self, self.sink, lambda: self.now)

    def sink(self, payload):
        details = {k: v for k, v in payload.items() if k not in BASE}
        self.events.append((payload['event'], details))

    def ready(self):
        return True

    def keep_still(self):
        pass

    def close(self):
        if self.goal is not None:
            self.goal.cancel()

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        if self.goal is not None and not self.goal.done:
            self.goal.done, self.goal.status = True, 'ABORTED'  # preempted
        self.goal = Goal(route, precise)
        self.goal.why = why
        self.goals.append(self.goal)
        return self.goal

    def drive(self):
        goal = self.goal
        if goal is None or goal.done:
            return
        x, y, yaw = self.pose
        # Nav2 passes via-poses within a metre; the last one must be reached.
        while len(goal.route) > 1 and math.dist(goal.route[0][:2], (x, y)) < 1.0:
            goal.route.pop(0)
        tx, ty, tyaw = goal.route[0]
        gap = math.dist((tx, ty), (x, y))
        step = self.speed * self.dt
        if gap <= step:
            x, y = tx, ty
            if len(goal.route) == 1:
                self.pose = (x, y, tyaw)
                goal.done, goal.status = True, 'SUCCEEDED'
                return
            goal.route.pop(0)
        else:
            yaw = math.atan2(ty - y, tx - x)
            x, y = x + step * math.cos(yaw), y + step * math.sin(yaw)
        self.path += min(gap, step)
        self.pose = (x, y, yaw)

    def look(self):
        x, y, yaw = self.pose
        for i, (label, ox, oy) in enumerate(self.course):
            bearing = math.atan2(oy - y, ox - x) - yaw
            if (math.dist((ox, oy), (x, y)) <= self.fov[1]
                    and abs(math.atan2(math.sin(bearing), math.cos(bearing))) <= self.fov[0] / 2):
                self.seen[(i, label, ox, oy)] = self.now

    def step(self, now):
        """Move the boat on to ``now``; return its pose and the tracks seen so far."""
        self.now = now
        moving = self.goal is not None and not self.goal.done
        self.drive()
        if moving and (self.goal.done or self.goal is None):
            self.stops += 1
        self.look()
        return self.pose, [Track(label, x, y, 0.9, None, True, False, str(i))
                           for (i, label, x, y), at in self.seen.items()
                           if self.now - at <= hold_over(label)]

    def run(self, seconds=1800.0):
        for tick in range(int(seconds / self.dt)):
            pose, tracks = self.step(tick * self.dt)
            outcome = self.host.tick(pose, tracks, self.fov)
            if outcome is not None:
                return outcome, self.now
        return 'RUNNING', seconds

    def names(self):
        """List the trees' own events: no goals, decisions or the host's start and finish."""
        return [event for event, _ in self.events
                if event not in ('goal', 'decision', 'task_started', 'home', 'task_finished')]


NO_ROS = """
import importlib.abc, sys
class NoRos(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in ('ament_index_python', 'rclpy'):
            raise ModuleNotFoundError(name)
sys.meta_path.insert(0, NoRos())
from mhseals_nav.tasks.host import config_path, load_params, MissionHost
print(config_path('mission_2026.yaml'))
print(sorted(load_params('mission', config_path('mission_2026.yaml'))['tasks']))
"""


def test_the_host_finds_its_config_with_no_ros_on_the_host():
    """The fast sim imports the host on a bare host: no ament index, no rclpy."""
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join(sys.path)}
    out = subprocess.run([sys.executable, '-c', NO_ROS], env=env, capture_output=True,
                         text=True, check=True).stdout.splitlines()
    assert out[0].endswith('/config/tasks/mission_2026.yaml')  # the source tree's
    assert 'docking' in out[1]  # and it read


def mission(path):
    return load_params('mission', config_path(path))


def check(world, outcome, elapsed):
    names = world.names()
    assert outcome == 'SUCCESS', [d for e, d in world.events if e == 'decision'][-6:]
    assert 'stage_skipped' not in names
    assert names.count('stage_completed') == 5
    assert names.count('gate_cleared') >= 2 + 4 + 1 + 1
    docked = next(d for e, d in world.events if e == 'docked')
    assert docked['banner'] == 'blue_square' and docked['bay'] == pytest.approx((14.0, -20.0))
    assert next(d for e, d in world.events if e == 'species_count')['count'] == 3
    assert next(d for e, d in world.events if e == 'spill_count')['count'] == 2
    assert 'speed_finished' in names and names[-2:] == ['returned_home', 'stage_completed']
    return elapsed


def test_full_mission_with_the_day_layout_runs_every_task_in_succession():
    world = Course(mission('mission_crane.yaml'))
    elapsed = check(world, *world.run())
    # One route per visible chain of gates: the 6 gates of channel and path are driven
    # as a handful of goals, not one stop per gate.
    gates = [g for g in world.goals if len(g.route) >= 4]
    assert gates and elapsed < 900


def test_full_mission_without_a_layout_searches_and_recognises_its_way():
    """With no approaches, tasks are found by search and recognition alone.

    Without the day's layout (or a course area) the dock and speed course can lie
    out of reach of a bounded search, so only the required stages must succeed here;
    every stage must still be attempted in order, each search explained.
    """
    world = Course(mission('mission.yaml'))
    outcome, elapsed = world.run()
    names = world.names()
    assert outcome == 'SUCCESS' and names[-2:] == ['returned_home', 'stage_completed']
    assert names.count('stage_started') == 5
    assert 'speed_finished' in names  # found by its mark
    reasons = [d['reason'] for e, d in world.events if e == 'decision']
    assert any(r.startswith('red_circle not seen') for r in reasons)
    assert elapsed < 1400


# crane_ml course-gen seed 1 (2026 rules) in the start frame: test/fixtures/course_2026_s1.json.
ELEMENTS_2026 = json.loads(
    (Path(__file__).parent / 'fixtures/course_2026_s1.json').read_text())['elements']
COURSE_2026 = [tuple(e[:3]) for e in ELEMENTS_2026]
SWAP = {'red_pole_buoy': 'green_pole_buoy', 'green_pole_buoy': 'red_pole_buoy',
        'red_buoy': 'green_buoy', 'green_buoy': 'red_buoy'}
# The same course with side S on the other side: mirrored across the start line, the
# gates' colours swapped so red stays to port, lights and banners as they were.
MIRRORED_2026 = [(SWAP.get(label, label) if role == 'gate' else label, x, -y)
                 for label, x, y, _task, role in ELEMENTS_2026]


@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_full_2026_mission_finds_and_completes_every_task_by_search(course, side):
    """The 2026 course with no layout: every stage by search, in the fixed order.

    Side S is unknown at the start: the start look finds the marina on either side,
    and the speed challenge is searched for on the side the marina fixed.
    """
    world = Course(mission('mission_2026.yaml'), course=course)
    outcome, elapsed = world.run()
    names = world.names()
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    assert outcome == 'SUCCESS', decisions[-8:]
    # Only the supply drop is skipped: no delivery actuator is fitted yet.
    skipped = [d['stage'] for e, d in world.events if e == 'stage_skipped']
    assert skipped == ['supply_drop'], decisions
    assert 'supply_drop: capability off (water_gun, ball_dropper)' in decisions
    assert names.count('stage_completed') == 5
    # Marina: the lowest-numbered free green slip is N3, banner_1 (truth 3.39, 12.84).
    docked = next(d for e, d in world.events if e == 'docked')
    assert docked['bay_number'] == 1 and docked['bay_colour'] == 'green'
    assert docked['bay'] == pytest.approx((3.39, side * 12.842))
    # Debris: both green beacons circled, and the debris positions reported.
    assert sum(1 for g in world.goals if g.why.startswith('round the beacon')) >= 4
    reported = {tuple(d['position']) for e, d in world.events if e == 'debris'}
    assert len(reported) >= 8
    # Speed: the red indicator turns the lap counter-clockwise, round the turn mark and
    # not the harbor buoy off the marina.
    assert any(r.startswith('red indicator: counter') for r in decisions)
    started = next(d for e, d in world.events if e == 'speed_started')
    assert math.dist(started['pose'][:2], (48.0, side * 40.0)) < 3.0
    assert names[-2:] == ['returned_home', 'stage_completed']
    assert elapsed < 600


@pytest.mark.parametrize('landmarks', [True, False], ids=['remembered', 'forgotten'])
def test_the_marina_found_at_the_start_is_docked_at_field_speed(landmarks):
    """At 0.4 m/s the marina, seen at the start look, is docked at ~10 min.

    The tracker has dropped its banners and lights 10 s after they left the view
    (``hold_over``), so only the host's landmark memory keeps the slips: without it,
    docking fails.
    """
    params = mission('mission_2026.yaml')
    if not landmarks:
        params['docking'].pop('landmarks')
    world = Course(params, course=COURSE_2026, speed=0.4)
    outcome, _ = world.run()
    docked = [d for e, d in world.events if e == 'docked']
    assert outcome == 'SUCCESS'
    if landmarks:
        assert [(d['bay_colour'], d['bay_number']) for d in docked] == [('green', 1)]
    else:
        assert docked == [] and ('stage_skipped', {'stage': 'docking'}) in [
            (e, {'stage': d.get('stage')}) for e, d in world.events if e == 'stage_skipped']


def test_a_short_run_skips_the_task_worth_least_per_second_and_still_gets_out():
    """With 700 s, 460 s are left after the exit's reserve.

    The debris task and the speed run fit (the most points per second), the marina
    does not; the exit always runs.
    """
    params = dict(mission('mission_2026.yaml'), timeout=700.0)
    world = Course(params, course=COURSE_2026)
    outcome, elapsed = world.run()
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    skipped = [d['stage'] for e, d in world.events if e == 'stage_skipped']
    assert outcome == 'SUCCESS' and 'docking' in skipped, decisions
    assert any(r.startswith('no time for docking') for r in decisions)
    assert 'speed_finished' in world.names()
    assert world.names()[-2:] == ['returned_home', 'stage_completed']


@pytest.mark.parametrize('spacing', [20.1, 25.0, 30.0, 40.0])
def test_the_channel_is_entered_through_both_gates_however_far_apart(spacing):
    """The second entry gate may lie past the camera's 20 m range from the first.

    The channel is two gates: after the first, the boat searches forward until it sees
    and crosses the second, rather than calling the channel done after one.
    """
    red, green = ELEMENTS_2026[0][:3], ELEMENTS_2026[1][:3]
    course = [red, green, ('red_pole_buoy', red[1] + spacing, 1.36),
              ('green_pole_buoy', green[1] + spacing, -1.36)]
    params = dict(mission('mission_2026.yaml'), tasks=['nav_channel'])
    world = Course(params, course=course)
    outcome, _ = world.run(300.0)
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    cleared = [d['position'] for e, d in world.events if e == 'gate_cleared']
    assert outcome == 'SUCCESS' and len(cleared) == 2, decisions
    assert cleared[1][0] == pytest.approx(red[1] + spacing, abs=0.5)


def test_a_channel_short_of_its_second_gate_is_never_claimed_done():
    """With gate 2 past the whole forward search, one gate fails the channel."""
    red, green = ELEMENTS_2026[0][:3], ELEMENTS_2026[1][:3]
    course = [red, green, ('red_pole_buoy', red[1] + 90.0, 1.36),
              ('green_pole_buoy', green[1] + 90.0, -1.36)]
    world = Course(dict(mission('mission_2026.yaml'), tasks=['nav_channel']), course=course)
    outcome, _ = world.run(300.0)
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    assert outcome != 'SUCCESS' and 'stage_completed' not in world.names()
    assert any(r.startswith('1 of 2 gates: none ahead') for r in decisions), decisions
