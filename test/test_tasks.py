import importlib.util
import math

import pytest

from mhseals_nav.tasks import geometry
from mhseals_nav.tasks.tracks import Track

# Never a module-level importorskip: Jazzy's launch_testing plugin turns it into a skip
# of the whole test directory.
pytestmark = pytest.mark.skipif(importlib.util.find_spec('py_trees') is None,
                                reason='needs py_trees')
if importlib.util.find_spec('py_trees') is not None:
    import py_trees
    from mhseals_nav.tasks.trees import TREES


def test_gates_pair_nearest_markers_and_face_away_from_the_boat():
    reds = [(10, 1), (20, 1)]
    greens = [(10, -1), (20, -1.5), (50, 9)]
    gates = geometry.gate_midpoints(reds, greens, origin=(0, 0, 0))
    assert [g[:2] for g in gates] == [(10, 0), (20, -0.25)]
    assert all(abs(g[2]) < 0.1 for g in gates)  # facing +x, through the gate
    behind = geometry.gate_midpoints(reds[:1], greens[:1], origin=(30, 0, math.pi))
    assert abs(abs(behind[0][2]) - math.pi) < 1e-9
    assert geometry.gate_midpoints([(0, 0)], [(20, 0)], (0, 0, 0), max_width=8) == []
    # One colour: each black buoy is used once, and never with itself.
    blacks = [(-2.5, 1.6), (-2.5, -1.6), (10, 12)]
    home = geometry.gate_midpoints(blacks, blacks, (6, 0, math.pi), max_width=4)
    assert [g[:2] for g in home] == [(-2.5, 0)] and abs(abs(home[0][2]) - math.pi) < 1e-9


def test_next_gate_is_ahead_and_not_cleared():
    gates = [(5, 0, 0), (12, 1, 0), (-4, 0, 0)]
    assert geometry.next_gate(gates, (0, 0, 0), []) == (5, 0, 0)
    assert geometry.next_gate(gates, (6, 0, 0), [(5, 0, 0)]) == (12, 1, 0)
    # Behind the direction of travel, or past the horizon, is not next.
    assert geometry.next_gate(gates, (13, 0, 0), [(5, 0, 0), (12, 1, 0)]) is None
    assert geometry.next_gate([(40, 0, 0)], (0, 0, 0), []) is None
    assert geometry.through((5, 0, 0)) == [(3.5, 0, 0), (6.1, 0, 0)]


def test_crossed_means_past_the_line_near_the_middle():
    gate = (10, 0, 0)
    assert geometry.crossed(gate, (11, 0.5, 0)) and not geometry.crossed(gate, (10.2, 0, 0))
    assert not geometry.crossed(gate, (11, 4, 0)) and not geometry.crossed(gate, (9, 0, 0))


def test_unique_spin_and_memory():
    assert geometry.unique([(0, 0), (0.5, 0), (3, 0)]) == [(0, 0), (3, 0)]
    turns = geometry.spin((1, 2, 0), 2)
    assert len(turns) == 8 and turns[-1][2] == pytest.approx(0, abs=1e-9)
    memory = geometry.remember([], [('red', 1, 1)], now=0.0)
    memory = geometry.remember(memory, [('red', 1.2, 1)], now=2.0)  # the same buoy, moved
    assert memory == [('red', 1.2, 1, 2.0)]
    assert geometry.remember(memory, [], now=4.0) == memory  # a short dropout keeps it
    assert geometry.remember(memory, [], now=8.0) == []


def test_dock_bays_open_toward_the_boat():
    banners = [(10, -5), (10, -3), (10, -1)]
    opening = geometry.dock_axis(banners, (10, -3), (0, 0, 0))
    assert opening == pytest.approx(math.pi)  # the spine runs along y; the boat is at -x
    assert geometry.dock_axis([(10, -3)], (10, -3), (0, -3, 0)) == pytest.approx(math.pi)
    poses = geometry.bay((10, -3), opening)
    assert poses['berth'] == pytest.approx((9.15, -3, 0))  # facing the spine, +x
    assert poses['staging'][:2] == pytest.approx((7, -3))


def test_circle_and_reverse_and_relative():
    poses = geometry.circle((0, 0), 2, 4, clockwise=False, start=(-5, 0))
    assert [tuple(round(v, 6) for v in p[:2]) for p in poses] == [
        (0, -2), (2, 0), (0, 2), (-2, 0)]
    assert poses[0][2] == pytest.approx(0)  # counterclockwise at the bottom faces +x
    assert geometry.circle((0, 0), 2, 4, True, (-5, 0))[0][:2] == pytest.approx((0, 2))
    back = geometry.reverse([(0, 0, 0), (5, 0, 0)])
    assert back == [(5, 0, pytest.approx(math.pi)), (0, 0, pytest.approx(math.pi))]
    route = geometry.relative((1, 1, math.pi / 2), [(2, 0), (2, 3)])
    assert route[0][:2] == pytest.approx((1, 3)) and route[1][:2] == pytest.approx((-2, 3))
    assert geometry.forward((0, 0, math.pi / 2), 3)[:2] == pytest.approx((0, 3))


class Goal:
    def __init__(self, world, route, precise=False, ticks=3, status='SUCCEEDED'):
        self.world, self.route, self.precise = world, route, precise
        self.ticks, self.cancelled, self.status = ticks, False, status
        self.sent, self.arrives = world.get('tick', 0), None  # arrives: even if not SUCCEEDED

    @property
    def done(self):
        # The boat arrives a tick before Nav2 reports success, as it would for real.
        left = self.ticks - (self.world.get('tick', 0) - self.sent)
        if left <= 1 and not self.cancelled and (self.arrives or self.status == 'SUCCEEDED'):
            self.world['pose'] = self.route[-1]
        elif left == 2 and len(self.route) > 1 and not self.cancelled:
            self.world['pose'] = self.route[0]  # on the way, at its first pose
        return left <= 0 or self.cancelled

    def cancel(self):
        self.cancelled = True


class World:
    """Ticks a task tree with a fake Nav2 and scripted object sightings."""

    def __init__(self, task, params):
        py_trees.blackboard.Blackboard.clear()  # one run per process, as for real
        self.state = dict(objects=[], pose=(0.0, 0.0, 0.0))
        self.events, self.goals = [], []
        self.tree = py_trees.trees.BehaviourTree(TREES[task](params))
        self.tree.setup(navigator=self, report=lambda event, **details: self.events.append(
            (event, details)))
        self.board = py_trees.blackboard.Client(name='world')
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.board.home = self.board.coverage = None
        self.board.tracks, self.board.fov = [], (1.9, 20.0)

    def hold(self):
        self.holds = getattr(self, 'holds', 0) + 1

    def send(self, route, precise=False, quiet=False, why=''):
        self.goals.append(Goal(self.state, route, precise))
        return self.goals[-1]

    def run(self, ticks=400, script=None):
        for tick in range(ticks):
            self.state['tick'] = tick
            if script:
                script(tick, self)
            self.board.objects, self.board.pose, self.board.now = (
                self.state['objects'], self.state['pose'], tick / 10)
            self.board.tracks = [Track(label, x, y, 1.0, None, True, False, '')
                                 for label, x, y in self.state['objects']]
            if self.board.home is None:
                self.board.home = self.state['pose']
            self.tree.tick()
            if self.tree.root.status in (py_trees.common.Status.SUCCESS,
                                         py_trees.common.Status.FAILURE):
                return self.tree.root.status.name
        return 'RUNNING'

    def names(self):
        # Navigation goals are reported too, but these tests follow the task's own events.
        return [event for event, _ in self.events if event not in ('goal', 'decision')]

    def event(self, name):
        return next(details for event, details in self.events if event == name)


def params(task):
    import yaml
    from pathlib import Path
    return yaml.safe_load((Path(__file__).parents[1] / f'config/tasks/{task}.yaml').read_text())


def gate(x, y, width, red='red_pole_buoy', green='green_pole_buoy'):
    return [(red, x, y + width / 2), (green, x, y - width / 2)]


def test_waiting_holds_station_against_drift_then_gives_up():
    from mhseals_nav.tasks.behaviours import WaitFor
    world = World('nav_channel', params('nav_channel'))
    world.tree = py_trees.trees.BehaviourTree(WaitFor('gate', ('red',), timeout=30.0))
    world.tree.setup(navigator=world, report=lambda event, **details: None)

    def drift(tick, world):  # the current carries the boat while nothing is in sight
        x, y, yaw = world.state['pose']
        world.state['pose'] = (x, y + 0.05, yaw)

    assert world.run(script=drift) == 'FAILURE'  # after the timeout
    assert world.goals and all(goal.route[-1][:2] == pytest.approx((0, 0.05))
                               for goal in world.goals)  # back to where it began waiting
    assert abs(world.state['pose'][1]) < 2.0
    assert world.holds > 0  # zero velocity between re-goals, not free drift


def test_nav_channel_crosses_both_gates():
    world = World('nav_channel', params('nav_channel'))
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3)
    assert world.run() == 'SUCCESS'
    assert world.names() == ['gate_cleared', 'gate_cleared']
    # One flowing route through both gates: no stop between them.
    assert len(world.goals) == 1
    route = [p[:2] for p in world.goals[0].route]
    assert route == [(5.5, 0.0), (8.1, 0.0), (11.5, 0.0), (14.1, 0.0)]


def test_nav_channel_searches_ahead_for_the_second_gate():
    world = World('nav_channel', params('nav_channel'))
    world.state['objects'] = gate(7, 0, 3)

    def appear(tick, world):
        if len(world.goals) == 2:  # the second gate shows up while searching ahead
            world.state['objects'] = gate(7, 0, 3) + gate(20, 0, 3)

    assert world.run(script=appear) == 'SUCCESS'
    assert world.goals[1].cancelled  # the search leg was preempted by the sighting
    assert world.goals[-1].route[-1][:2] == pytest.approx((21.1, 0))


def test_follow_path_runs_every_gate_and_reports_the_species_count():
    world = World('follow_path', params('follow_path'))
    world.state['objects'] = (
        gate(5, 0, 2.4, 'red_buoy', 'green_buoy') + gate(10, -2, 2.4, 'red_buoy', 'green_buoy')
        + gate(14.5, -5, 2.4, 'red_buoy', 'green_buoy')
        + [('yellow_buoy', 7.5, 0.5), ('yellow_buoy', 12.5, -2.5), ('yellow_buoy', 12.7, -2.6)])
    assert world.run() == 'SUCCESS'
    assert world.names().count('gate_cleared') == 3
    assert world.event('species_count') == {'count': 2, 'method': 'display'}


def test_docking_skips_the_occupied_bay():
    world = World('docking', params('docking'))
    world.state['objects'] = [('blue_square', 10, -1), ('red_circle', 10, -3),
                              ('blue_square', 10, -5), ('yellow_target_boat', 9.25, -1)]
    assert world.run() == 'SUCCESS'
    berth = next(goal for goal in world.goals if goal.precise)
    assert berth.route[-1] == pytest.approx((8.8, -5, 0))
    assert world.names() == ['docked', 'undocked']
    assert world.goals[-1].route[-1][:2] == pytest.approx((7, -5))  # backed out


def test_speed_challenge_circles_the_mark_and_counts_spills_past_the_gate():
    world = World('speed_challenge', dict(params('speed_challenge'), approach=[0, 0, 0]))
    start = gate(5, 0, 2.5, 'red_buoy', 'green_buoy') + [('black_buoy', 2, 3)]
    world.state['objects'] = list(start)

    def mark(tick, world):
        if tick == 5:  # seen while driving on from the gate
            world.state['objects'] = start + [
                ('black_buoy', 12, 1.5), ('black_buoy', 15, -1.5), ('blue_buoy', 20, 0)]

    assert world.run(script=mark) == 'SUCCESS'
    names = world.names()
    assert names.index('speed_started') < names.index('spill_count') < names.index('speed_finished')
    assert world.event('spill_count')['count'] == 2  # the one before the gate is not in the task
    # The lap and the return through the gate are one route: no stop at the mark.
    lap = next(goal.route for goal in world.goals if len(goal.route) == 10)
    assert all(math.dist(p[:2], (20, 0)) == pytest.approx(2.5) for p in lap[:8])
    assert lap[-1][0] < 5 and abs(abs(lap[-1][2]) - math.pi) < 1e-9
    assert world.goals[-1].route is lap
    assert world.goals[0].route == [(0, 0, 0)]  # its approach


def test_return_home_finds_the_black_gate_by_home():
    world = World('return_home', params('return_home'))
    world.board.home = (0.0, 0.0, 0.0)
    world.state['pose'] = (20.0, 0.0, math.pi)
    world.state['objects'] = [('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6),
                              ('black_buoy', 10, 12.6), ('black_buoy', 14, 9.4)]
    assert world.run() == 'SUCCESS'
    assert world.goals[0].route == [pytest.approx((6, 0, math.pi))]
    assert world.goals[-1].route[-1][:2] == pytest.approx((-3.6, 0))
    assert world.names() == ['gate_cleared', 'returned_home']


def test_mission_skips_a_failed_task_and_still_returns_home():
    settings = dict(tasks=['nav_channel', 'follow_path', 'return_home'])
    for name in settings['tasks']:
        settings[name] = params(name)
    settings['follow_path']['time_limit'] = 3.0  # no path in sight: it times out
    world = World('mission', settings)
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3) + [
        ('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6)]
    assert world.run(ticks=800) == 'SUCCESS'
    names = world.names()
    assert world.event('stage_skipped')['stage'] == 'follow_path'
    assert names[-2:] == ['returned_home', 'stage_completed']


def test_mission_needs_the_channel():
    settings = dict(tasks=['nav_channel', 'return_home'])
    for name in settings['tasks']:
        settings[name] = params(name)
    settings['nav_channel']['time_limit'] = 2.0
    world = World('mission', settings)
    assert world.run() == 'FAILURE'


def test_task_labels_are_detector_or_course_classes():
    from pathlib import Path
    import yaml
    root = Path(__file__).parents[1]
    loaded = yaml.safe_load((root / 'config/custom_object_detection.yaml').read_text())
    classes = loaded.get('/**', {}).get('ros__parameters', loaded)
    detector = {value['label'] for key, value in classes['object_detection'].items()
                if key.startswith('class_')}
    banners = {f'{c}_{s}' for c in ('red', 'green', 'blue')
               for s in ('circle', 'triangle', 'square', 'plus')}
    vessels = {'yellow_target_boat', 'black_target_boat'}
    for task in ('nav_channel', 'follow_path', 'docking', 'speed_challenge',
                 'return_home', 'pair_test', 'circle_test'):
        config = params(task)
        labels = {config[k] for k in ('red', 'green', 'count', 'mark', 'marker', 'banner')
                  if k in config}
        labels |= set(config.get('banners', [])) | set(config.get('vessels', []))
        assert labels <= detector | banners | vessels, (task, labels - detector)


def test_a_task_drives_to_its_approach_first():
    settings = dict(params('docking'), approach=[12.0, -14.0, -1.57])
    world = World('docking', settings)
    world.state['objects'] = [('blue_square', 10, -1), ('red_circle', 10, -3),
                              ('blue_square', 10, -5)]
    assert world.run() == 'SUCCESS'
    assert world.goals[0].route == [(12.0, -14.0, -1.57)]


def test_docking_looks_along_the_dock_when_no_empty_bay_is_in_view():
    world = World('docking', params('docking'))
    occupied = [('blue_square', 10, -1), ('yellow_target_boat', 9.25, -1)]
    world.state['objects'] = list(occupied)

    def turn(tick, world):
        if any(len(goal.route) == 1 and goal.route[0][:2] == world.goals[0].route[-1][:2] for goal in world.goals[1:]):  # the sweep shows the rest
            world.state['objects'] = occupied + [('red_circle', 10, -3), ('blue_square', 10, -5)]

    assert world.run(script=turn) == 'SUCCESS'
    berth = next(goal for goal in world.goals if goal.precise)
    assert berth.route[-1][:2] == pytest.approx((8.8, -5))


@pytest.mark.parametrize('swapped', [False, True])
@pytest.mark.parametrize('yaw', [0.0, 1.2, -2.0])
def test_pair_test_crosses_both_ways_and_turns_beyond_the_pair(swapped, yaw):
    settings = params('pair_test')
    world = World('pair_test', settings)
    start = (4, -3, yaw)
    centre = geometry.forward(start, 7)
    sign = -1 if swapped else 1
    dx, dy = -1.5 * math.sin(yaw) * sign, 1.5 * math.cos(yaw) * sign
    world.state.update(pose=start, objects=[
        ('red_pole_buoy', centre[0] + dx, centre[1] + dy),
        ('green_pole_buoy', centre[0] - dx, centre[1] - dy),
    ])
    assert world.run() == 'SUCCESS'
    assert world.names() == ['gate_cleared', 'pair_turned', 'pair_returned']
    outward, clearance, turn, back = [goal.route for goal in world.goals]
    assert outward == [pytest.approx(p) for p in geometry.through(centre)]
    far = geometry.forward(centre, settings['clearance'])
    assert clearance == [pytest.approx(far)]
    assert turn[0][:2] == pytest.approx(far[:2])
    assert math.cos(turn[0][2] - yaw) == pytest.approx(-1)
    assert world.goals[2].precise  # finish the in-place turn, separately
    assert back[0] == pytest.approx(turn[0])
    assert back[-1][:2] == pytest.approx(geometry.forward(centre, -1.5)[:2])


def test_pair_test_remembers_the_pair_after_tracks_disappear():
    world = World('pair_test', params('pair_test'))
    world.state['objects'] = gate(7, 0, 3)

    def disappear(tick, world):
        if world.goals:
            # A different pair appears after the outbound goal was chosen.
            world.state['objects'] = gate(20, 10, 3) if tick % 2 else []

    assert world.run(script=disappear) == 'SUCCESS'
    assert world.goals[-1].route[-1][:2] == pytest.approx((5.5, 0))


@pytest.mark.parametrize('objects', [[], [('red_pole_buoy', 7, 1.5)],
                                      gate(7, 0, 0.5), gate(7, 0, 12)])
def test_pair_test_fails_after_bounded_search_without_a_valid_pair(objects):
    settings = params('pair_test')
    world = World('pair_test', settings)
    world.state['objects'] = objects
    assert world.run() == 'FAILURE'
    assert not world.names()
    positions = [(0, 0)] + [goal.route[-1][:2] for goal in world.goals]
    distance = sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))
    assert distance == pytest.approx(settings['search_distance'])


@pytest.mark.parametrize('clockwise', [False, True])
def test_circle_test_stages_then_completes_a_lap_and_restores_home(clockwise):
    settings = dict(params('circle_test'), circle_clockwise=clockwise)
    world = World('circle_test', settings)
    home, centre = (2, -4, 0.7), (12, -4)
    world.state.update(pose=home, objects=[('yellow_buoy', *centre)])

    def disappear(tick, world):
        if world.goals:
            world.state['objects'] = []  # survive a camera dropout

    assert world.run(script=disappear) == 'SUCCESS'
    assert world.names() == ['buoy_circled', 'returned_home']
    entry, first, second, returned = [goal.route for goal in world.goals]
    assert entry[0][:2] == pytest.approx((9, -4))
    lap = entry + first + second
    assert len(first) == len(second) == settings['circle_points'] // 2
    assert all(math.dist(p[:2], centre) == pytest.approx(3) for p in lap)
    angles = [geometry.heading(centre, pose) for pose in lap]
    swept = sum(math.remainder(b - a, 2 * math.pi)
                for a, b in zip(angles, angles[1:]))
    assert swept == pytest.approx((-1 if clockwise else 1) * 2 * math.pi)
    assert returned == [home] and world.goals[-1].precise
    assert world.state['pose'] == home


def test_circle_test_keeps_the_origin_from_before_searching():
    world = World('circle_test', params('circle_test'))
    home = (3, -2, 0.3)
    world.state['pose'] = home

    def appear(tick, world):
        if world.state['pose'] != home:
            x, y, _ = geometry.forward(world.state['pose'], 8)
            world.state['objects'] = [('yellow_buoy', x, y)]

    assert world.run(script=appear) == 'SUCCESS'
    assert len(world.goals[0].route) == 1  # one search leg, not stop-and-go steps
    assert world.goals[-1].route == [home]


def test_circle_test_fails_if_search_only_finds_other_buoys():
    settings = params('circle_test')
    world = World('circle_test', settings)
    world.state['objects'] = [('blue_buoy', 8, 0), ('red_pole_buoy', 6, 2)]
    assert world.run() == 'FAILURE'
    assert not world.names()
    assert world.state['pose'][0] == pytest.approx(settings['search_distance'])


@pytest.mark.parametrize('task,goal_index', [
    ('pair_test', 1), ('pair_test', 2), ('pair_test', 3),
    ('circle_test', 0), ('circle_test', 1), ('circle_test', 2),
    ('circle_test', 3),
])
def test_commissioning_nav_failure_prevents_completion(task, goal_index):
    world = World(task, params(task))
    world.state['objects'] = gate(7, 0, 3) + [('yellow_buoy', 12, 0)]

    def abort(tick, world):
        if len(world.goals) > goal_index:
            world.goals[goal_index].status = 'CANCELED'  # not retried

    assert world.run(script=abort) == 'FAILURE'
    completion = 'pair_returned' if task == 'pair_test' else 'returned_home'
    assert completion not in world.names()


@pytest.mark.parametrize('task', ['pair_test', 'circle_test'])
def test_commissioning_tasks_cancel_navigation_when_stopped(task):
    world = World(task, params(task))
    world.state['objects'] = gate(7, 0, 3) + [('yellow_buoy', 12, 0)]
    assert world.run(ticks=1) == 'RUNNING'
    world.tree.root.stop(py_trees.common.Status.INVALID)
    assert world.goals[-1].cancelled


# Audit fixes: failure handling, lost tracks, partial detections, retries, flow.

def test_crossing_between_ticks_counts():
    gate = (10, 0, 0)
    assert geometry.crossed(gate, (11, 3.5, 0), previous=(9, -1, 0))  # met the line at 1.25
    assert not geometry.crossed(gate, (11, 6, 0), previous=(9, 5, 0))  # passed beside it


def test_gate_run_retries_an_aborted_gate_and_keeps_a_lost_gate():
    world = World('nav_channel', params('nav_channel'))
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3)

    def trouble(tick, world):
        if len(world.goals) == 1:
            world.goals[0].status = 'ABORTED'  # Nav2 gives up once
            world.goals[0].ticks = 20
        if len(world.goals) == 2:
            world.state['objects'] = []  # and the tracks drop out

    assert world.run(script=trouble) == 'SUCCESS'
    assert world.names() == ['gate_cleared', 'gate_cleared']
    assert ('decision', dict(kind='retry', reason='gate 1 aborted: retry')) in world.events
    assert world.goals[1].route[-1][:2] == pytest.approx((14.1, 0))


def test_gate_run_gives_up_after_its_retries():
    world = World('nav_channel', params('nav_channel'))
    world.state['objects'] = gate(7, 0, 3)

    def abort(tick, world):
        for goal in world.goals:
            goal.status, goal.ticks = 'ABORTED', 20  # never reaches the gate

    assert world.run(script=abort) == 'FAILURE'
    assert sum(1 for e, d in world.events if d.get('kind') == 'retry') == 2


def test_channel_with_one_colour_in_view_searches_instead_of_waiting():
    world = World('nav_channel', params('nav_channel'))
    world.state['objects'] = [('red_pole_buoy', 7, 1.5)]

    def green(tick, world):
        if len(world.goals) == 2:  # the search turned the camera onto the green pole
            world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3)

    assert world.run(script=green) == 'SUCCESS'
    assert world.goals[0].route[-1][:2] != pytest.approx((0, 0))  # it moved to look
    assert world.names() == ['gate_cleared', 'gate_cleared']


def test_mission_retries_a_failed_task_then_skips_it_with_reasons():
    settings = dict(tasks=['nav_channel', 'docking', 'return_home'])
    for name in settings['tasks']:
        settings[name] = params(name)
    settings['docking'].update(attempts=2, search={'enable': dict(
        priors=False, scan=False, next_view=False, sweep=False)})
    world = World('mission', settings)
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3) + [
        ('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6)]
    assert world.run(ticks=1500) == 'SUCCESS'
    failed = [d for e, d in world.events if e == 'stage_failed']
    assert [d['attempt'] for d in failed] == [1, 2]
    reasons = [d['reason'] for e, d in world.events if e == 'decision']
    assert 'docking failed: attempt 2' in reasons
    assert 'docking failed 2x: giving up' in reasons
    assert 'skipping docking: on to return_home' in reasons
    assert all(len(r) <= 60 for r in reasons)


def test_berth_counts_when_nav2_gives_up_inside_the_bay_and_keeps_station():
    world = World('docking', params('docking'))
    world.state['objects'] = [('blue_square', 10, -1), ('red_circle', 10, -3),
                              ('blue_square', 10, -5)]

    def stubborn(tick, world):
        for goal in world.goals:
            if goal.precise:
                goal.status, goal.arrives = 'ABORTED', True  # waves upset the stopped checker
        docked = any(e == 'docked' for e, _ in world.events)
        if docked and not any(e == 'undocked' for e, _ in world.events):
            x, y, yaw = world.state['pose']
            world.state['pose'] = (x - 0.1, y, yaw)  # wind pushes it out

    assert world.run(script=stubborn) == 'SUCCESS'
    assert world.names() == ['docked', 'undocked']
    berth = next(goal.route[-1] for goal in world.goals if goal.precise)
    assert any(goal.route == [pytest.approx(berth)] and not goal.precise
               for goal in world.goals)  # drove back in while holding
    assert world.holds > 0  # and otherwise held zero velocity


def test_spin_report_turns_one_quarter_goal_at_a_time():
    settings = dict(params('follow_path'), report='spin')
    world = World('follow_path', settings)
    world.state['objects'] = gate(5, 0, 2.4, 'red_buoy', 'green_buoy') + [
        ('yellow_buoy', 7.5, 0.5)]
    assert world.run() == 'SUCCESS'
    turns = [goal.route for goal in world.goals if len(goal.route) == 1
             and goal.route[0][:2] == pytest.approx(world.state['pose'][:2])]
    assert len(turns) >= 4  # four quarter turns for one buoy


def test_a_task_approach_is_its_own_and_does_not_leak_to_the_next():
    settings = dict(tasks=['nav_channel', 'docking', 'speed_challenge'])
    for name in settings['tasks']:
        settings[name] = params(name)
    settings['docking'].update(approach=[10.0, -5.0, 0.0], search={'enable': dict(
        priors=False, scan=False, next_view=False, sweep=False)})  # nothing there: skipped
    world = World('mission', settings)
    speed = gate(20, 11, 2.5, 'red_buoy', 'green_buoy') + [('blue_buoy', 36, 11)]
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3) + speed
    world.run(ticks=1500)
    assert 'speed_started' in world.names()  # its gate was not filtered by the dock's area


def test_recognised_task_is_approached_without_a_layout():
    from mhseals_nav.tasks.behaviours import PlanHypothesis
    world = World('nav_channel', params('nav_channel'))
    world.board.register_key(key='hypotheses', access=py_trees.common.Access.WRITE)
    world.board.hypotheses = [dict(type='dock', state='confirmed', confidence=0.9,
                                   pose=[12.0, -20.0, -math.pi / 2])]
    world.tree = py_trees.trees.BehaviourTree(PlanHypothesis('dock', 'view', 'dock'))
    world.tree.setup(navigator=world, report=lambda event, **details: None)
    world.board.register_key(key='view', access=py_trees.common.Access.READ)
    assert world.run(ticks=2) == 'SUCCESS'
    assert world.board.view == [pytest.approx((12.0, -14.0, -math.pi / 2))]


def test_a_replanned_gate_route_never_turns_back_to_a_passed_lead_in():
    world = World('nav_channel', params('nav_channel'))
    world.state.update(pose=(6.5, 0.0, 0.0), objects=gate(7, 0, 3) + gate(13, 0, 3))
    world.run(ticks=1)
    assert world.goals[0].route[0][:2] == pytest.approx((8.1, 0))  # not back to 5.5


def test_conditions_gate_precise_tasks_and_catch_a_weak_thruster():
    from mhseals_nav.tasks.conditions import Conditions
    c = Conditions()
    assert c.precise() is None  # nothing known: never blocks
    c.fix = 5
    assert c.precise() is None  # float: dock from perception
    c.fix = 4
    assert 'GPS fix 4' in c.precise()
    c = Conditions(dict(float_policy='skip'))
    c.fix = 5
    assert c.precise()
    c = Conditions()
    for t in range(12):
        c.drift(0.5, float(t))
    assert 'drift' in c.precise()
    c.drift(0.1, 12.0)
    assert c.precise() is None
    for t in range(7):  # off the command while saturated
        c.tracking((0.4, 0.0), (0.1, 0.0), (1.0, 0.0, 0.2), float(t))
    assert c.thrusters()
    c.tracking((0.4, 0.0), (0.1, 0.0), (0.5, 0.0, 0.2), 8.0)  # not saturated: just slow
    assert c.thrusters() is None


def test_mission_skips_docking_without_rtk_and_aborts_a_task_on_weak_thrusters():
    from mhseals_nav.tasks.conditions import Conditions
    settings = dict(tasks=['nav_channel', 'docking', 'return_home'])
    for name in settings['tasks']:
        settings[name] = params(name)
    world = World('mission', settings)
    world.board.register_key(key='conditions', access=py_trees.common.Access.WRITE)
    world.board.conditions = conditions = Conditions()
    conditions.fix = 3
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3) + [
        ('blue_square', 10, -5), ('black_buoy', -2.5, 1.6), ('black_buoy', -2.5, -1.6)]
    assert world.run(ticks=1500) == 'SUCCESS'
    assert 'docked' not in world.names()
    reasons = [d['reason'] for e, d in world.events if e == 'decision']
    assert 'no-go: GPS fix 3 < 6' in reasons

    world = World('mission', settings)
    world.board.register_key(key='conditions', access=py_trees.common.Access.WRITE)
    world.board.conditions = conditions = Conditions()
    world.state['objects'] = gate(7, 0, 3) + gate(13, 0, 3)

    def weaken(tick, world):
        conditions.weak_s = 6.0 if tick > 0 else 0.0

    assert world.run(ticks=60, script=weaken) == 'FAILURE'  # the channel is mandatory
    assert 'no-go: thrusters saturate, velocity off' in [
        d['reason'] for e, d in world.events if e == 'decision']
    assert world.goals and all(goal.cancelled for goal in world.goals)  # stopped at once
    assert 'gate_cleared' not in world.names()


def replacing_driver(refuse):
    """A Driver on a fake Nav2; ``refuse`` (a one-item list) rejects the goals sent."""
    from types import SimpleNamespace
    from mhseals_nav.tasks.behaviours import Hold
    goals = []

    def send(route, precise=False, quiet=False, why=''):
        goal = SimpleNamespace(route=route, done=refuse[0], cancelled=False,
                               status='REJECTED' if refuse[0] else None)
        goal.cancel = lambda: setattr(goal, 'cancelled', True)
        goals.append(goal)
        return goal

    driver = Hold('drive', 10.0)  # any Driver
    driver.setup(navigator=SimpleNamespace(send=send), report=None)
    driver.board = SimpleNamespace(now=0.0, pose=(0.0, 0.0, 0.0))
    driver.initialise()
    driver.drive([(0, 0, 0), (5, 0, 0)])
    assert driver.driving() == 'RUNNING' and len(goals) == 1
    return driver, goals


def test_a_replaced_goal_is_cancelled_when_its_replacement_is_rejected():
    refuse = [False]
    driver, goals = replacing_driver(refuse)
    refuse[0] = True  # Nav2 refuses the new route on every resend
    driver.drive([(0, 0, 0), (9, 0, 0)])
    assert driver.driving() == 'REJECTED' and len(goals) > 2
    assert goals[0].cancelled  # the old route does not drive on alone


def test_a_replaced_goal_is_cancelled_when_driving_stops_mid_handover():
    driver, goals = replacing_driver([False])
    driver.drive([(0, 0, 0), (9, 0, 0)])  # same action: the running goal is not cancelled
    assert len(goals) == 2 and not goals[0].cancelled
    driver.stop_driving()  # an override while the replacement is in flight
    assert goals[0].cancelled and goals[1].cancelled
