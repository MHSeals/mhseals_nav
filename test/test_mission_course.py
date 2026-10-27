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
from mhseals_nav.startup_projection import effective_limits
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


def field():
    """Return Course keywords for the boat as the launch resolves it: speed and turn rate."""
    limits = effective_limits()
    (vx, vy, wz), back = limits['velocity']['max'], -limits['velocity']['min'][0]
    checkers = limits['goal_checkers']
    return {'speed': vx, 'omni': (vy, back, wz, {
        False: checkers['general_goal_checker']['yaw'],
        True: checkers['precise_goal_checker']['yaw']})}


class Course:
    """A boat that drives Nav2 routes at ``speed`` and a camera with a cone of view.

    The tracks it hands the host are the object tracker's: everything seen, until the
    tracker's hold-over for its class runs out (``hold_over``).
    """

    def __init__(self, params, dt=0.5, speed=1.0, hfov=math.radians(110), range_m=20.0,
                 course=COURSE, host=True, omni=None):
        self.dt, self.speed, self.fov, self.course = dt, speed, (hfov, range_m), course
        self.omni = omni
        py_trees.blackboard.Blackboard.clear()  # one run per process, as for real
        self.pose, self.seen, self.goal, self.now = (0.0, 0.0, 0.0), {}, None, 0.0
        self.events, self.goals, self.stops, self.path = [], [], 0, 0.0
        self.trace = []  # (time, metres moved) per step
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
        if self.omni:
            return self.drive_omni(goal)
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

    def drive_omni(self, goal):
        """Drive as the holonomic boat does: ``omni`` = (vy, vx astern, wz, {precise: yaw tol}).

        It translates any way (forward at ``speed``, sideways at vy, astern at vx_back,
        an ellipse between) while it turns at wz: toward the way it goes, and to the
        goal's heading for the last metre, which the goal checker holds to its tolerance.
        """
        vy, back, wz, tolerance = self.omni
        x, y, yaw = self.pose
        while len(goal.route) > 1 and math.dist(goal.route[0][:2], (x, y)) < 1.0:
            goal.route.pop(0)
        tx, ty, tyaw = goal.route[0]
        final, gap = len(goal.route) == 1, math.dist((tx, ty), (x, y))
        way = math.atan2(ty - y, tx - x) if gap > 1e-6 else tyaw
        want = tyaw if final and gap < 1.0 else way
        turn = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
        yaw += max(-wz * self.dt, min(wz * self.dt, turn))
        off = way - yaw
        ahead = self.speed if math.cos(off) >= 0 else back
        step = self.dt / math.hypot(math.cos(off) / ahead, math.sin(off) / vy)
        moved = min(gap, step)
        if gap > 1e-6:
            x, y = x + moved * math.cos(way), y + moved * math.sin(way)
        self.path += moved
        self.pose = (x, y, yaw)
        if gap <= step and not final:
            goal.route.pop(0)
        elif gap <= step and abs(math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))) <= (
                tolerance[goal.precise]):
            goal.done, goal.status = True, 'SUCCEEDED'

    def stalls(self, limit, speed=0.05):
        """Return the (start, seconds) of every span over ``limit`` s moving under ``speed``."""
        spans, since = [], None
        for now, moved in self.trace + [(self.now + self.dt, math.inf)]:
            if moved < speed * self.dt:
                since = now if since is None else since
            elif since is not None:
                if now - since > limit:
                    spans.append((since, now - since))
                since = None
        return spans

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
        moving, travelled = self.goal is not None and not self.goal.done, self.path
        self.drive()
        self.trace.append((now, self.path - travelled))
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


NO_PY_TREES = """
import importlib.abc, sys
class NoPyTrees(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in ('py_trees', 'numpy', 'ament_index_python', 'rclpy'):
            raise ModuleNotFoundError(name)
sys.meta_path.insert(0, NoPyTrees())
from mhseals_nav.tasks.budget import choose
from mhseals_nav.tasks.params import config_path, load_params
params = load_params('mission', config_path('mission_2026.yaml'), 'finals')
print(params['timeout'], params['exit_reserve'], params['speed_challenge']['expected_s'])
task = {'name': 'a', 'points': 1, 'expected_s': 50}
print(load_params('mission')['timeout'], *choose(60.0, [task], {}))
"""


def test_the_run_budget_resolves_with_no_py_trees_on_the_host():
    """A harness reads the run budget, the profile's params and the budget's choice alone."""
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join(sys.path)}
    out = subprocess.run([sys.executable, '-c', NO_PY_TREES], env=env, capture_output=True,
                         text=True, check=True).stdout.split()
    assert [float(v) for v in out[:4]] == [720.0, 300.0, 380.0, 720.0] and out[4] == 'a'


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
SEED_2026 = [tuple(e[:3]) for e in ELEMENTS_2026]
# Seed 1 with a boat in green slip 1, 1.2 m out from banner_1 toward the far pier: the
# piers' slips open toward each other, as the far pier's boat stands from its banner_2
# and the docked hull from banner_3. The run sees it only at the start look, so docking
# starts on remembered slips and must find it taken at the staging pose.
SLIP_1_BOAT = ('boat', 2.419, 13.546)
# Truth slip centres of the near pier (crane seed 1, 1.45 m out from each banner).
SLIPS_2026 = {1: (2.22, 13.69), 2: (1.03, 12.05)}
# The logic tests run the course with time to spare; the timeslot's budget has its own.
UNHURRIED = 1800.0
COURSE_2026 = SEED_2026 + [SLIP_1_BOAT]
SWAP = {'red_pole_buoy': 'green_pole_buoy', 'green_pole_buoy': 'red_pole_buoy',
        'red_buoy': 'green_buoy', 'green_buoy': 'red_buoy'}
# The same course with side S on the other side: mirrored across the start line, the
# gates' colours swapped so red stays to port, lights and banners as they were.
MIRRORED_2026 = [(SWAP.get(label, label) if role == 'gate' else label, x, -y)
                 for label, x, y, _task, role in ELEMENTS_2026 + [[*SLIP_1_BOAT, '', '']]]


@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_full_2026_mission_finds_and_completes_every_task_by_search(course, side):
    """The 2026 course with no layout: every stage by search, in the fixed order.

    Side S is unknown at the start: the start look finds the marina on either side,
    and the speed challenge is searched for on the side the marina fixed.
    """
    world = Course(dict(mission('mission_2026.yaml'), timeout=UNHURRIED), course=course)
    outcome, elapsed = world.run()
    names = world.names()
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    assert outcome == 'SUCCESS', decisions[-8:]
    # Only the supply drop is skipped: no delivery actuator is fitted yet.
    skipped = [d['stage'] for e, d in world.events if e == 'stage_skipped']
    assert skipped == ['supply_drop'], decisions
    assert 'supply_drop: capability off (water_gun, ball_dropper)' in decisions
    assert names.count('stage_completed') == 5
    # Marina: green slip 1 holds a boat, so the lowest-numbered free green slip is
    # slip 2 (banner_2, truth 2.20, 11.20).
    docked = next(d for e, d in world.events if e == 'docked')
    assert docked['bay_number'] == 2 and docked['bay_colour'] == 'green'
    assert docked['bay'] == pytest.approx((2.199, side * 11.2))
    # In the slip, on the side it opens to (toward the far pier), not behind the pier.
    slip = (SLIPS_2026[2][0], side * SLIPS_2026[2][1])
    assert math.dist(docked['bay_slip'], slip) < 0.5
    berth = next(g.route[-1] for g in world.goals if g.why == 'slide into the bay')
    assert math.dist(berth[:2], slip) < 0.5
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
    params = dict(mission('mission_2026.yaml'), timeout=UNHURRIED)
    if not landmarks:
        params['docking'].pop('landmarks')
    world = Course(params, course=COURSE_2026, speed=0.4)
    outcome, _ = world.run()
    docked = [d for e, d in world.events if e == 'docked']
    assert outcome == 'SUCCESS'
    if landmarks:
        # The boat in slip 1 was forgotten by docking: slip 1 was chosen from memory,
        # then found taken at its staging pose.
        assert [(d['bay_colour'], d['bay_number']) for d in docked] == [('green', 2)]
        assert 'green slip 2: the chosen one is not free' in [
            d['reason'] for e, d in world.events if e == 'decision']
    else:
        assert docked == [] and ('stage_skipped', {'stage': 'docking'}) in [
            (e, {'stage': d.get('stage')}) for e, d in world.events if e == 'stage_skipped']


class Changing(Course):
    """A course that changes (``change``: old element -> new) as the boat lines up on a slip."""

    def __init__(self, params, change, **kwargs):
        super().__init__(params, **kwargs)
        self.change = change

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        if why.startswith('line up on') and self.change:
            self.course = [self.change.get(e, e) for e in self.course]
            self.course += [new for old, new in self.change.items() if old is None]
            self.change = None
        return super().send(route, precise, quiet, why, reverse)


GREEN_1 = ('green_light_beacon', 3.654, 13.206)


@pytest.mark.parametrize('change', [{None: SLIP_1_BOAT},
                                    {GREEN_1: ('red_light_beacon', *GREEN_1[1:])}],
                         ids=['a-boat-moves-in', 'the-light-turns-red'])
def test_a_slip_taken_while_the_boat_lines_up_is_not_entered(change):
    """Slip 1 is free and green when docking starts, then not as the boat lines up on it.

    At the staging pose, with the slip in view, the choice is made again: the boat
    lines up on green slip 2 instead and docks there, never driving into slip 1.
    """
    world = Changing(dict(mission('mission_2026.yaml'), timeout=UNHURRIED), change,
                     course=SEED_2026)
    outcome, _ = world.run()
    docked = [(d['bay_colour'], d['bay_number']) for e, d in world.events if e == 'docked']
    slips = [d['reason'] for e, d in world.events if e == 'decision' and d['kind'] == 'slip']
    assert outcome == 'SUCCESS' and docked == [('green', 2)], slips
    assert slips == ['green slip 1', 'green slip 2: the chosen one is not free']
    berths = [g.route[-1][:2] for g in world.goals if g.why == 'slide into the bay']
    assert berths and all(math.dist(b, SLIPS_2026[1]) > 1.5 for b in berths)
    assert math.dist(berths[-1], SLIPS_2026[2]) < 0.5


def test_a_failed_mandatory_task_still_goes_home():
    """With no channel to pass, the run still goes home, and fails.

    The mandatory first task fails; the exit runs all the same.
    """
    params = mission('mission_crane.yaml')
    params['nav_channel']['time_limit'] = 60.0
    world = Course(params, course=[o for o in COURSE if 'pole' not in o[0]])
    outcome, _ = world.run()
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    assert outcome == 'FAILURE' and 'course failed: on to return_home' in decisions
    assert world.names()[-2:] == ['returned_home', 'stage_completed']
    started = [d['stage'] for e, d in world.events if e == 'stage_started']
    assert started == ['nav_channel', 'return_home']


def test_the_host_hands_the_trees_its_actuators_and_none_is_on_by_default():
    from mhseals_nav.tasks.actuators import Unavailable
    from mhseals_nav.tasks.behaviours import Deliver
    params = mission('mission_2026.yaml')
    world = Course(params, course=COURSE_2026)
    assert all(isinstance(a, Unavailable) for a in world.host.actuators.values())
    assert sorted(world.host.actuators) == ['ball_dropper', 'horn', 'water_gun']
    gun = object()
    params['capabilities'] = dict(params['capabilities'], water_gun=True)
    host = MissionHost('mission', params, world, world.sink, lambda: world.now,
                       actuators={'water_gun': gun, 'ball_dropper': object()})
    delivers = [b for b in host.tree.root.iterate() if isinstance(b, Deliver)]
    assert delivers and all(d.actuator is gun for d in delivers)
    assert isinstance(host.actuators['ball_dropper'], Unavailable)  # given, but off


@pytest.mark.parametrize('profile, done, skipped', [
    ('qualifying', ['follow_path', 'docking'], ['speed_challenge', 'supply_drop']),
    ('finals', ['follow_path'], ['docking', 'speed_challenge', 'supply_drop'])])
@pytest.mark.parametrize('course', [COURSE_2026, MIRRORED_2026], ids=['port', 'starboard'])
def test_the_timeslot_budget_at_field_speed(profile, done, skipped, course):
    """A 900 s slot less 180 s of set-up leaves a 720 s run, which the run states.

    At 0.4 m/s it holds the channel, the debris field and the marina, not the speed
    run as well: an optional task starts only if it fits before the exit's reserve, by
    points per second. Finals keeps a minute more for the exit and plans the marina
    with a margin, so it heads home after the debris field.
    """
    params = load_params('mission', config_path('mission_2026.yaml'), profile)
    world = Course(params, course=course, speed=0.4)
    outcome, elapsed = world.run()
    started = next(d for e, d in world.events if e == 'task_started')
    assert started['time_limit_s'] == params['timeout'] == 720.0
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS' and completed == ['nav_channel', *done, 'return_home']
    assert [d['stage'] for e, d in world.events if e == 'stage_skipped'] == skipped
    assert all(any(r.startswith(f'no time for {task}') or r.startswith(f'{task}: capability')
                   for r in decisions) for task in skipped)
    # Home with the exit reserve's margin to spare, never on the limit.
    assert elapsed < 720.0 - 60.0


def test_a_timeslot_sets_the_run_budget_and_a_profile_changes_only_known_keys(tmp_path):
    finals = load_params('mission', config_path('mission_2026.yaml'), 'finals')
    assert finals['timeout'] == 720.0 and finals['profile'] == 'finals'
    assert finals['exit_reserve'] == 300.0 and finals['docking']['attempts'] == 1
    assert finals['docking']['berth'] == 1.3  # untouched keys stay the mission's
    assert finals['docking']['search']['regions']  # merged key by key
    path = tmp_path / 'mission.yaml'
    path.write_text('tasks: [return_home]\ntimeslot: {slot_s: 900.0, setup_reserve_s: 300.0}\n'
                    'exit_reserve: 240.0\n')
    assert load_params('mission', str(path))['timeout'] == 600.0
    path.write_text('tasks: [return_home]\ntimeslot: {slot_s: 900.0, setup_reserve_s: 700.0}\n'
                    'exit_reserve: 240.0\n')
    with pytest.raises(ValueError):
        load_params('mission', str(path))
    path.write_text('tasks: [return_home]\ntimeout: 600.0\nprofile: finals\n')
    with pytest.raises(KeyError, match='exit_reserve is not a mission key'):
        load_params('mission', str(path))


@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_the_sweep_mows_the_speed_region_on_the_side_the_marina_fixed(course, side):
    """With no views left (none here), the sweep mows for the turn mark.

    Side S is known from the marina, so the lawnmower covers the speed challenge's
    region on that side only, never the mirrored half of the course box.
    """
    params = mission('mission_2026.yaml')
    speed = params['speed_challenge']
    params['speed_challenge'] = dict(speed, search=dict(speed['search'], enable={
        **speed['search']['enable'], 'next_view': False, 'head_for': False}))
    world = Course(dict(params, timeout=UNHURRIED), course=course, speed=0.4)
    outcome, _ = world.run()
    sweeps = [g.route for g in world.goals if 'sweep' in g.why]
    assert outcome == 'SUCCESS' and sweeps
    assert all(10.0 - 1e-6 <= side * y <= 60.0 and 15.0 <= x <= 80.0
               for route in sweeps for x, y, _ in route)


@pytest.mark.parametrize('profile', ['finals', 'qualifying'])
@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_a_seen_speed_gate_leads_the_views_on_to_its_turn_mark_at_field_speed(
        course, side, profile):
    """At 0.4 m/s the views pass the speed gate with its mark beyond camera range.

    The mark lies 10-33 m past its gate, so a gate seen first is a cue: the views go on
    through it toward the mark. Finals has no sweep to fall back on, so without the cue
    the views run out and the speed run is given up; qualifying has one, and the cue
    finds the mark before it is reached.
    """
    params = load_params('mission', config_path('mission_2026.yaml'), profile)
    world = Course(dict(params, timeout=UNHURRIED), course=course, speed=0.4)
    outcome, _ = world.run()
    decisions = [d['reason'] for e, d in world.events if e == 'decision']
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS' and 'speed_challenge' in completed, decisions[-8:]
    assert not any('sweep' in g.why for g in world.goals)
    gate = (48.0, side * 40.0)
    started = next(d for e, d in world.events if e == 'speed_started')
    assert math.dist(started['pose'][:2], gate) < 3.0
    # A view from the gate or past it (away from the start) found the mark, or the
    # layout's guess of the task already led the boat to where it saw it.
    views = [g.route[-1] for g in world.goals if g.why.startswith(('next view', 'head for past'))]
    assert any(math.dist(v[:2], gate) < 15.0 and side * v[1] >= side * gate[1] - 1.0
               for v in views) or (not views and any(
                   g.why.startswith('head for the likely speed') for g in world.goals))


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


ROTATED = ['nav_channel', 'speed_challenge', 'follow_path', 'docking', 'supply_drop',
           'return_home']


@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_the_speed_run_before_the_debris_field_is_not_led_off_by_its_gates(course, side):
    """The rotated order: the speed run comes before the debris field, at field speed.

    The debris channel's red/green gates lie straight ahead, not yet cleared (so not
    excluded); they are no cue for the turn mark, being outside the speed challenge's
    region on the course side. The views go to the speed gate, never past the debris.
    """
    params = dict(load_params('mission', config_path('mission_2026.yaml'), 'qualifying'),
                  tasks=ROTATED, agenda=[], timeout=UNHURRIED)
    world = Course(params, course=course, **field())
    outcome, _ = world.run()
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS' and completed[:2] == ['nav_channel', 'speed_challenge']
    started = next(d for e, d in world.events if e == 'speed_started')
    assert math.dist(started['pose'][:2], (48.0, side * 40.0)) < 3.0
    searched = [g.route[-1] for g in world.goals if g.why.startswith(('next view', 'head'))]
    assert searched and all(side * y > 5.0 for _, y, _ in searched)


@pytest.mark.parametrize('course, side', [(COURSE_2026, 1), (MIRRORED_2026, -1)],
                         ids=['side-port', 'side-starboard'])
def test_the_layout_plans_the_order_and_the_boat_never_stalls_long(course, side):
    """At the field's speed and turn rate, every scoring task, in the order planned.

    The boat turns at wz while it moves (``field``): the start look stops once the marina
    fixes the side and does not turn back; the speed run heads for where the layout puts
    it, then past the gate it saw; the marina, beside the way out, comes last. Before
    (fixed order, stop-turn-look) this course took 952 s with a 15 s start stall.
    """
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    world = Course(dict(params, timeout=UNHURRIED), course=course, **field())
    outcome, elapsed = world.run()
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS'
    assert completed == ['nav_channel', 'follow_path', 'speed_challenge', 'docking',
                         'return_home']
    assert any(g.why.startswith('head for the likely speed') for g in world.goals)
    # The start look (one side, or both) is the longest stop; the rest are short.
    assert not world.stalls(13.0) and all(at < 20.0 for at, _ in world.stalls(6.0))
    assert elapsed < 870.0


@pytest.mark.parametrize('course', [COURSE_2026, MIRRORED_2026], ids=['port', 'starboard'])
def test_every_task_fits_the_timeslot_at_the_speed_policys_transit_speed(course):
    """Fit every scoring task into qualifying's 720 s run at 0.6 m/s.

    That is the speed policy's open-water pace, under the 0.8 m/s ceiling. The exit's
    reserve is sized from the far corner of each task's region at the measured speed:
    home at ~612 s on both sides, ~108 s to spare.
    """
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    world = Course(params, course=course, **dict(field(), speed=0.6))
    outcome, elapsed = world.run()
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS' and len(completed) == 5, completed
    assert elapsed < 660.0


@pytest.mark.parametrize('course, done', [
    (COURSE_2026, []), (MIRRORED_2026, ['docking'])], ids=['port', 'starboard'])
def test_at_field_speed_the_corner_safe_reserve_skips_what_may_not_fit(course, done):
    """At 0.4 m/s in 720 s the far-corner reserve skips the speed run on both sides.

    Port also skips the marina (home at ~408 s); starboard docks (home at ~596 s).
    """
    params = load_params('mission', config_path('mission_2026.yaml'), 'qualifying')
    world = Course(params, course=course, **dict(field(), speed=0.4))
    outcome, elapsed = world.run()
    completed = [d['stage'] for e, d in world.events if e == 'stage_completed']
    assert outcome == 'SUCCESS'
    assert completed == ['nav_channel', 'follow_path', *done, 'return_home']
    assert elapsed < 660.0
