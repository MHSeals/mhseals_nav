"""Task startup must wait for TF even when Nav2 and detections are ready."""

import importlib.util
import json
from types import SimpleNamespace

import pytest

# Never a module-level importorskip: Jazzy's launch_testing plugin turns it into a skip
# of the whole test directory.
pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('rclpy') is None or importlib.util.find_spec('py_trees') is None,
    reason='needs rclpy and py_trees')
if not pytestmark.args[0]:
    import rclpy
    from mhseals_nav.tasks.interface import TaskRunner


def test_circle_waits_for_initial_tf_before_planning(monkeypatch):
    """A late TF subscriber must not fail a task with an already-visible buoy."""
    rclpy.init()
    runner = None
    goals = []

    def send(route, precise=False, quiet=False, why=''):
        goals.append(route)
        return SimpleNamespace(done=False, cancel=lambda: None)

    try:
        runner = TaskRunner('circle_test', dict(
            mark='yellow_buoy', search_distance=15.0, circle_radius=3.0,
            circle_clockwise=False, circle_points=12))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', send)
        monkeypatch.setattr(runner, 'pose', lambda: None)
        for _ in range(3):
            runner.tick()
        assert runner.outcome is None and runner.started is None
        assert runner.board.home is None and not goals

        home = (1.0, 0.0, 0.5)
        monkeypatch.setattr(runner, 'pose', lambda: home)
        runner.tick()
        assert runner.outcome is None and runner.started is not None
        assert runner.board.home == home
        assert goals[0][0][:2] == pytest.approx((6.0, 0.0))
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_home_reported_once_and_goal_per_send(monkeypatch):
    """/task/events carries home once and one goal per route, not per retry."""
    rclpy.init()
    runner = None
    try:
        runner = TaskRunner('circle_test', dict(
            mark='yellow_buoy', search_distance=15.0, circle_radius=3.0,
            circle_clockwise=False, circle_points=12))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        events = []
        monkeypatch.setattr(runner, 'report', lambda event, **d: events.append((event, d)))
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send',
                            lambda route, precise=False, quiet=False, why='': SimpleNamespace(
                                done=False, cancel=lambda: None))
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 2.0, 0.5))
        for _ in range(3):
            runner.tick()
        homes = [d for e, d in events if e == 'home']
        assert homes == [dict(x=1.0, y=2.0)]
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def reporting_navigator(events):
    """A real Navigator whose Nav2 clients are stubbed, so ``send`` reports for real."""
    from mhseals_nav.tasks.interface import Navigator
    navigator = Navigator.__new__(Navigator)
    navigator.report = lambda event, **d: events.append((event, d))
    navigator.pose = lambda x, y, yaw: (x, y, yaw)
    client = SimpleNamespace(send_goal_async=lambda goal: SimpleNamespace(
        done=lambda: False, result=lambda: None, add_done_callback=lambda f: None))
    navigator.to_pose = navigator.through = client
    return navigator


def board(route, pose=(0.0, 0.0, 0.0)):
    return SimpleNamespace(get=lambda key: route, exists=lambda key: True, now=0.0, pose=pose)


def test_navigate_reports_one_goal_across_retries(monkeypatch):
    from mhseals_nav.tasks import interface
    from mhseals_nav.tasks.behaviours import Navigate
    refused = SimpleNamespace(done=True, status='REJECTED', cancel=lambda: None)
    monkeypatch.setattr(interface, 'Goal', lambda future: refused)
    monkeypatch.setattr(interface, 'NavigateToPose', SimpleNamespace(Goal=lambda **k: None))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    events = []
    node = Navigate('go', 'route', retries=0)
    node.navigator = reporting_navigator(events)
    node.report = lambda event, **d: events.append((event, d))
    node.board = board([(0.0, 0.0, 0.0), (4.0, 5.0, 1.5)])
    node.initialise()
    for _ in range(40):
        node.update()
    assert events == [('goal', dict(x=4.0, y=5.0, yaw=1.5, why='go'))]


def test_navigate_on_an_empty_route_fails_without_event_or_raising():
    from py_trees.common import Status
    from mhseals_nav.tasks.behaviours import Navigate
    events = []
    node = Navigate('go', 'route')
    node.navigator = SimpleNamespace(send=lambda route, precise=False, quiet=False, why='': None)
    node.report = lambda event, **d: events.append(event)
    node.board = board([])
    node.initialise()
    assert node.update() == Status.FAILURE and events == []


def test_empty_route_send_reports_nothing(monkeypatch):
    from mhseals_nav.tasks import interface
    monkeypatch.setattr(interface, 'Goal', lambda future: SimpleNamespace(done=False))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    events = []
    reporting_navigator(events).send([])
    assert events == []


def test_pass_gates_through_a_gate_reports_one_goal(monkeypatch):
    from mhseals_nav.tasks import geometry, interface
    events = []
    navigator = reporting_navigator(events)
    monkeypatch.setattr(interface, 'Goal', lambda future: SimpleNamespace(done=False))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    navigator.send(geometry.through((10.0, 0.0, 0.0)))
    assert len(events) == 1 and events[0][0] == 'goal'


def authority(owner):
    """An authority record shaped like harness/authority.derive output."""
    return json.dumps(dict(schema='mhseals.authority/1', owner=owner, source=None,
                           claims=[], task=dict(state='RUNNING') if owner == 'task' else None))


def test_human_override_ends_the_task_at_once_and_never_resumes(monkeypatch):
    """An rc/keyboard/hold owner cancels the goal in the callback and ends the run."""
    from mhseals_nav.tasks.interface import EXIT_CODES, claim_suspended
    assert claim_suspended(authority('rc')) and claim_suspended(authority('keyboard'))
    assert claim_suspended(authority('hold'))
    for owner in ('agent', 'task', 'none', 'unknown'):
        assert not claim_suspended(authority(owner))
    for bad in ('', 'not json', 'null', '[]', '{}'):
        assert not claim_suspended(bad)

    rclpy.init()
    runner = None
    goals, cancels, events = [], [], []

    def send(route, precise=False, quiet=False, why=''):
        goals.append(route)
        return SimpleNamespace(done=False, cancel=lambda: cancels.append(1))

    try:
        runner = TaskRunner('circle_test', dict(
            mark='yellow_buoy', search_distance=15.0, circle_radius=3.0,
            circle_clockwise=False, circle_points=12))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', send)
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 0.0, 0.5))
        monkeypatch.setattr(runner, 'report', lambda event, **d: events.append(event))
        # A human owner before the start only delays it.
        runner._authority(SimpleNamespace(data=authority('hold')))
        runner.tick()
        assert runner.started is None and not goals
        runner._authority(SimpleNamespace(data=authority('none')))
        runner.tick()
        assert len(goals) == 1 and not cancels
        runner._authority(SimpleNamespace(data=authority('rc')))  # no tick needed
        assert cancels and runner.outcome == 'PREEMPTED' and 'task_suspended' in events
        assert EXIT_CODES['PREEMPTED'] == 3
        runner._authority(SimpleNamespace(data=authority('none')))
        for _ in range(3):
            runner.tick()
        assert len(goals) == 1 and runner.outcome == 'PREEMPTED'
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_cancel_before_acceptance_still_cancels():
    """A goal cancelled while its request is in flight is cancelled once accepted."""
    from mhseals_nav.tasks.interface import Goal
    callbacks, cancelled = [], []
    request = SimpleNamespace(add_done_callback=callbacks.append)
    goal = Goal(request)
    goal.cancel()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: SimpleNamespace(
        done=lambda: False), cancel_goal_async=lambda: cancelled.append(1))
    callbacks[0](SimpleNamespace(result=lambda: handle))
    assert cancelled == [1]


def test_status_follows_the_mission_and_stale_tracks_expire(monkeypatch):
    """The status carries the fixture's keys; tracks the tracker stopped sending go."""
    from pathlib import Path
    from mhseals_nav.tasks.tracks import Track
    fixture = json.loads((Path(__file__).parent / 'fixtures/mission_status.json').read_text())
    rclpy.init()
    runner = None
    try:
        params = dict(tasks=['nav_channel', 'return_home'], timeout=100.0)
        from mhseals_nav.tasks.interface import load_params
        for name in params['tasks']:
            params[name] = load_params(name)
        runner = TaskRunner('mission', params)
        published = []
        monkeypatch.setattr(runner.status, 'publish', lambda m: published.append(
            json.loads(m.data)))
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', lambda route, precise=False, quiet=False,
                            why='': SimpleNamespace(done=False, cancel=lambda: None))
        monkeypatch.setattr(runner, 'pose', lambda: (0.0, 0.0, 0.0))
        clock = [10.0]
        monkeypatch.setattr(runner, 'now', lambda: clock[0])
        runner._remember([Track('red_pole_buoy', 7.0, 1.5, 0.9, None, True, False, '3')])
        runner.tick()
        assert runner.objects == [('red_pole_buoy', 7.0, 1.5)]
        status = published[-1]
        assert set(fixture) <= set(status) and status['stage'] == 'nav_channel'
        assert status['stages']['nav_channel']['state'] == 'running'
        assert status['remaining'] == ['return_home']
        assert 'starting nav_channel' in [d['reason'] for d in status['recent']]
        clock[0] += runner.memory_horizon + 1  # the tracker went quiet
        runner.tick()
        assert runner.objects == [] and runner.board.objects == []
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()
