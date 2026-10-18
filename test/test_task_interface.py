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
