"""Task startup must wait for TF even when Nav2 and detections are ready."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('py_trees')

import rclpy  # noqa: E402

from mhseals_nav.tasks.interface import TaskRunner  # noqa: E402


def test_circle_waits_for_initial_tf_before_planning(monkeypatch):
    """A late TF subscriber must not fail a task with an already-visible buoy."""
    rclpy.init()
    runner = None
    goals = []

    def send(route, precise=False):
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
                            lambda route, precise=False: SimpleNamespace(
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


def test_navigate_reports_goal_once_per_route():
    from mhseals_nav.tasks.behaviours import Navigate
    sent, events = [], []
    node = Navigate('go', 'route')
    node.navigator = SimpleNamespace(send=lambda route, precise=False: sent.append(route))
    node.report = lambda event, **d: events.append((event, d))
    node.board = SimpleNamespace(get=lambda key: [(0.0, 0.0, 0.0), (4.0, 5.0, 1.5)], now=0.0)
    node.initialise()
    node.send()  # a retry
    assert len(sent) == 2
    assert events == [('goal', dict(x=4.0, y=5.0, yaw=1.5))]


def test_navigate_on_an_empty_route_fails_without_raising():
    from py_trees.common import Status
    from mhseals_nav.tasks.behaviours import Navigate
    events = []
    node = Navigate('go', 'route')
    node.navigator = SimpleNamespace(send=lambda route, precise=False: None)
    node.report = lambda event, **d: events.append(event)
    node.board = SimpleNamespace(get=lambda key: [], now=0.0)
    node.initialise()
    assert node.update() == Status.FAILURE and events == []
