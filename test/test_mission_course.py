"""The whole mission on crane_ml's roboboat_2025 layout, in a kinematic world.

The boat drives each Nav2 route at a steady speed, Nav2-style (via-poses within a
metre are passed), and the camera sees only what is inside its cone and range, as
the sim's would. It checks the task logic end to end: every stage in succession,
search when things are out of view, and no stop-and-go between gates.
"""

import importlib.util
import json
import math
from pathlib import Path

from mhseals_nav.tasks import load_config
from mhseals_nav.tasks.tracks import Track
import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('py_trees') is None,
                                reason='needs py_trees')
if importlib.util.find_spec('py_trees') is not None:
    import py_trees
    from mhseals_nav.tasks.recognition import Recogniser
    from mhseals_nav.tasks.search import Coverage
    from mhseals_nav.tasks.trees import TREES


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


class Goal:
    def __init__(self, route, precise):
        self.route, self.precise = [tuple(p) for p in route], precise
        self.done, self.status = False, None

    def cancel(self):
        if not self.done:
            self.done, self.status = True, 'CANCELED'


class Course:
    """A boat that drives Nav2 routes at ``speed`` and a camera with a cone of view."""

    def __init__(self, params, dt=0.5, speed=1.0, hfov=math.radians(110), range_m=20.0,
                 course=COURSE):
        self.dt, self.speed, self.fov, self.course = dt, speed, (hfov, range_m), course
        py_trees.blackboard.Blackboard.clear()  # one run per process, as for real
        self.pose, self.seen, self.goal = (0.0, 0.0, 0.0), {}, None
        self.events, self.goals, self.stops, self.path = [], [], 0, 0.0
        self.tree = py_trees.trees.BehaviourTree(TREES['mission'](params))
        self.tree.setup(navigator=self, report=lambda event, **d: self.events.append(
            (event, d)))
        self.board = py_trees.blackboard.Client(name='course')
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage',
                    'hypotheses'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)
        search = params.get('search') or {}
        self.board.home, self.board.fov = self.pose, self.fov
        self.board.coverage = Coverage(search.get('area'), 1.5, self.pose, 60.0, 120.0)
        self.recogniser = Recogniser(load_config('recognition'))

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
        for label, ox, oy in self.course:
            bearing = math.atan2(oy - y, ox - x) - yaw
            if (math.dist((ox, oy), (x, y)) <= self.fov[1]
                    and abs(math.atan2(math.sin(bearing), math.cos(bearing))) <= self.fov[0] / 2):
                self.seen[(label, ox, oy)] = True  # the tracker keeps static tracks

    def run(self, seconds=1800.0):
        for tick in range(int(seconds / self.dt)):
            now = tick * self.dt
            moving = self.goal is not None and not self.goal.done
            self.drive()
            if moving and (self.goal.done or self.goal is None):
                self.stops += 1
            self.look()
            objects = list(self.seen)
            tracks = [Track(label, x, y, 0.9, None, True, False, str(i))
                      for i, (label, x, y) in enumerate(objects)]
            self.board.coverage.observe(self.pose, self.fov, now)
            self.board.hypotheses = self.recogniser.update(tracks, self.board.home, now)
            self.board.objects, self.board.tracks = objects, tracks
            self.board.pose, self.board.now = self.pose, now
            self.tree.tick()
            if self.tree.root.status in (py_trees.common.Status.SUCCESS,
                                         py_trees.common.Status.FAILURE):
                return self.tree.root.status.name, now
        return 'RUNNING', seconds

    def names(self):
        return [event for event, _ in self.events if event not in ('goal', 'decision')]


def mission(path):
    params = load_config(Path(path).stem)
    for name in params['tasks']:
        params[name] = {**load_config(name), **params.get(name, {})}
    return params


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
    assert 'stage_skipped' not in names, decisions
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
