import math

import pytest

from mhseals_nav.tasks import geometry

py_trees = pytest.importorskip('py_trees')
from mhseals_nav.tasks.trees import TREES  # noqa: E402


def test_gates_pair_nearest_markers_and_face_away_from_the_boat():
    reds = [(10, 1), (20, 1)]
    greens = [(10, -1), (20, -1.5), (50, 9)]
    gates = geometry.gate_midpoints(reds, greens, origin=(0, 0, 0))
    assert [g[:2] for g in gates] == [(10, 0), (20, -0.25)]
    assert all(abs(g[2]) < 0.1 for g in gates)  # facing +x, through the gate
    behind = geometry.gate_midpoints(reds[:1], greens[:1], origin=(30, 0, math.pi))
    assert abs(abs(behind[0][2]) - math.pi) < 1e-9
    assert geometry.gate_midpoints([(0, 0)], [(20, 0)], (0, 0, 0), max_width=8) == []


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
    def __init__(self, world, route, ticks=3):
        self.world, self.route, self.ticks, self.cancelled = world, route, ticks, False

    @property
    def done(self):
        # The boat arrives a tick before Nav2 reports success, as it would for real.
        self.ticks -= 1
        if self.ticks <= 1 and not self.cancelled:
            self.world['pose'] = self.route[-1]
        return self.ticks <= 0

    status = 'SUCCEEDED'

    def cancel(self):
        self.cancelled = True


class World:
    """Ticks a task tree with a fake Nav2 and scripted object sightings."""

    def __init__(self, task, params):
        self.state = dict(objects=[], pose=(0.0, 0.0, 0.0))
        self.events, self.goals = [], []
        self.tree = py_trees.trees.BehaviourTree(TREES[task](params))
        self.tree.setup(navigator=self, report=lambda event, **details: self.events.append(
            (event, details)))
        self.board = py_trees.blackboard.Client(name='world')
        for key in ('objects', 'pose', 'now'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)

    def send(self, route):
        self.goals.append(Goal(self.state, route))
        return self.goals[-1]

    def run(self, ticks=200, script=None):
        for tick in range(ticks):
            if script:
                script(tick, self)
            self.board.objects, self.board.pose, self.board.now = (
                self.state['objects'], self.state['pose'], tick / 10)
            self.tree.tick()
            if self.tree.root.status in (py_trees.common.Status.SUCCESS,
                                         py_trees.common.Status.FAILURE):
                return self.tree.root.status.name
        return 'RUNNING'

    def names(self):
        return [event for event, _ in self.events]


GATES = dict(red='red_pole_buoy', green='green_pole_buoy', max_gate_width=8.0,
             search_distance=10.0)


def test_gates_with_both_gates_visible():
    world = World('gates', GATES)
    world.state['objects'] = [('red_pole_buoy', 10, 2), ('green_pole_buoy', 10, -2),
                              ('red_pole_buoy', 25, 2), ('green_pole_buoy', 25, -2)]
    assert world.run() == 'SUCCESS'
    assert len(world.goals) == 1 and len(world.goals[0].route) == 2
    assert world.names() == ['gate_cleared']


def test_gates_search_ahead_for_the_exit_gate():
    world = World('gates', GATES)
    entry = [('red_pole_buoy', 10, 2), ('green_pole_buoy', 10, -2)]
    exit_gate = [('red_pole_buoy', 30, 2), ('green_pole_buoy', 30, -2)]
    world.state['objects'] = list(entry)

    def appear(tick, world):
        if len(world.goals) == 2:  # the exit gate shows up while searching ahead
            world.state['objects'] = entry + exit_gate

    assert world.run(script=appear) == 'SUCCESS'
    assert world.goals[1].route[0][:2] == pytest.approx((20, 0))  # searched 10 m ahead
    assert world.goals[1].cancelled  # the search was preempted by the sighting
    assert world.goals[-1].route[0][:2] == pytest.approx((30, 0))
    assert world.names() == ['gate_cleared', 'gate_cleared']


DEBRIS = dict(red='red_buoy', green='green_buoy', channel_gates=4, max_gate_width=8.0,
              survivor=['green_light_buoy'], circle_radius=2.0, circle_clockwise=False,
              debris=['black_buoy'], beacons=['green_light_buoy', 'red_light_buoy'])


def test_debris_reports_sightings_circles_the_survivor_and_returns():
    world = World('debris', DEBRIS)
    world.state['objects'] = [('red_buoy', 5, 2), ('green_buoy', 5, -2),
                              ('red_buoy', 12, 2), ('green_buoy', 12, -2),
                              ('black_buoy', 20, 4), ('green_light_buoy', 25, 0)]
    assert world.run() == 'SUCCESS'
    names = world.names()
    assert names.count('debris') == 1 and names.count('beacon') == 1
    assert names.index('channel_transited') < names.index('survivor_circled') \
        < names.index('returned')
    assert world.goals[-1].route[-1][:2] == pytest.approx((5, 0))  # back through the first gate


def test_debris_without_survivor_still_returns():
    world = World('debris', DEBRIS)
    world.state['objects'] = [('red_buoy', 5, 2), ('green_buoy', 5, -2)]
    assert world.run() == 'SUCCESS'
    assert 'no_survivor_found' in world.names() and 'returned' in world.names()


SPRINT = dict(red='red_pole_buoy', green='green_pole_buoy', max_gate_width=8.0,
              search_distance=20.0, circle_radius=2.0,
              beacon_directions={'red_light_buoy': False, 'green_light_buoy': True})


def test_sprint_circles_by_beacon_colour_and_returns():
    world = World('sprint', SPRINT)
    world.state['objects'] = [('red_pole_buoy', 5, 2), ('green_pole_buoy', 5, -2)]

    def beacon(tick, world):
        if tick == 15:
            world.state['objects'] = world.state['objects'] + [('green_light_buoy', 30, 0)]

    assert world.run(script=beacon) == 'SUCCESS'
    found = dict(world.events)['beacon_found']
    assert found['circle_target'] == 'green_light_buoy'
    circle = next(goal.route for goal in world.goals if len(goal.route) == 6)
    # Clockwise around (30, 0): the second pose is clockwise from the first.
    a, b = (math.atan2(p[1], p[0] - 30) for p in circle[:2])
    assert math.remainder(b - a, 2 * math.pi) < 0
    assert world.names()[-1] == 'sprint_finished'


def test_speed_challenge_picks_the_route_by_colour():
    params = dict(routes={'Red': [[10, 0], [10, 10]], 'Green': [[10, 0], [10, -10]]})
    world = World('speed_challenge', params)
    world.state['objects'] = [('Green', 3, 0)]
    assert world.run() == 'SUCCESS'
    assert world.goals[0].route[-1][:2] == pytest.approx((10, -10))
    assert dict(world.events)['route_chosen']['route_choice'] == 'Green'
