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

CIRCLE = {'mark': 'yellow_buoy', 'search_distance': 15.0, 'circle_radius': 3.0,
          'circle_clockwise': False, 'circle_points': 12}


def test_circle_waits_for_initial_tf_before_planning(monkeypatch):
    """A late TF subscriber must not fail a task with an already-visible buoy."""
    rclpy.init()
    runner = None
    goals = []

    def send(route, precise=False, quiet=False, why=''):
        goals.append(route)
        return SimpleNamespace(done=False, cancel=lambda: None)

    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
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
        runner = TaskRunner('circle_test', dict(CIRCLE))
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
        assert homes == [{'x': 1.0, 'y': 2.0}]
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def reporting_navigator(events):
    """Return a real Navigator with stubbed Nav2 clients, so ``send`` reports for real."""
    from mhseals_nav.tasks.interface import Navigator
    navigator = Navigator.__new__(Navigator)
    navigator.report = lambda event, **d: events.append((event, d))
    navigator.pose = lambda x, y, yaw: (x, y, yaw)
    client = SimpleNamespace(send_goal_async=lambda goal: SimpleNamespace(
        done=lambda: False, result=lambda: None, add_done_callback=lambda f: None))
    navigator.to_pose = navigator.through = client
    navigator.goals, navigator.closed = [], False
    return navigator


def board(route, pose=(0.0, 0.0, 0.0)):
    return SimpleNamespace(get=lambda key: route, exists=lambda key: True, now=0.0, pose=pose)


def test_navigate_reports_one_goal_across_retries(monkeypatch):
    from mhseals_nav.tasks import interface
    from mhseals_nav.tasks.behaviours import Navigate
    refused = SimpleNamespace(done=True, status='REJECTED', cancel=lambda: None, settled=True)
    monkeypatch.setattr(interface, 'Goal', lambda future: refused)
    monkeypatch.setattr(interface, 'NavigateToPose', SimpleNamespace(Goal=lambda **k: None))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    events = []
    node = Navigate('go', 'route', retries=0)
    node.setup(navigator=reporting_navigator(events),
               report=lambda event, **d: events.append((event, d)))
    node.board = board([(0.0, 0.0, 0.0), (4.0, 5.0, 1.5)])
    node.initialise()
    for _ in range(40):
        node.update()
    assert events == [('goal', {'x': 4.0, 'y': 5.0, 'yaw': 1.5, 'why': 'go'})]


def test_navigate_on_an_empty_route_fails_without_event_or_raising():
    from py_trees.common import Status
    from mhseals_nav.tasks.behaviours import Navigate
    events = []
    node = Navigate('go', 'route')
    node.setup(navigator=SimpleNamespace(send=lambda route, precise=False, quiet=False,
                                         why='': None),
               report=lambda event, **d: events.append(event))
    node.board = board([])
    node.initialise()
    assert node.update() == Status.FAILURE and events == []


def test_empty_route_send_reports_nothing(monkeypatch):
    from mhseals_nav.tasks import interface
    monkeypatch.setattr(interface, 'Goal', lambda future: SimpleNamespace(
        done=False, settled=False))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    events = []
    reporting_navigator(events).send([])
    assert events == []


def test_pass_gates_through_a_gate_reports_one_goal(monkeypatch):
    from mhseals_nav.tasks import geometry, interface
    events = []
    navigator = reporting_navigator(events)
    monkeypatch.setattr(interface, 'Goal', lambda future: SimpleNamespace(
        done=False, settled=False))
    monkeypatch.setattr(interface, 'NavigateThroughPoses', SimpleNamespace(Goal=lambda **k: None))
    navigator.send(geometry.through((10.0, 0.0, 0.0)))
    assert len(events) == 1 and events[0][0] == 'goal'


def authority(owner):
    """Return an authority record shaped like harness/authority.derive output."""
    return json.dumps({'schema': 'mhseals.authority/1', 'owner': owner, 'source': None,
                       'claims': [],
                       'task': {'state': 'RUNNING'} if owner == 'task' else None})


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
        runner = TaskRunner('circle_test', dict(CIRCLE))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', send)
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 0.0, 0.5))
        monkeypatch.setattr(runner, 'report', lambda event, **d: events.append(event))
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


@pytest.mark.parametrize('owner', ['rc', 'keyboard', 'hold'])
def test_a_task_started_while_a_human_has_control_refuses_and_never_starts(
        monkeypatch, owner):
    from mhseals_nav.tasks.interface import EXIT_CODES
    rclpy.init()
    runner = None
    goals, events = [], []
    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', lambda *a, **k: goals.append(a))
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 0.0, 0.5))
        monkeypatch.setattr(runner, 'report', lambda event, **d: events.append((event, d)))
        runner._authority(SimpleNamespace(data=authority(owner)))
        assert runner.outcome == 'REFUSED' and EXIT_CODES['REFUSED'] == 4
        assert ('task_refused', {'owner': owner}) in events
        assert f'{owner} has control: not starting' in [
            d.get('reason') for e, d in events if e == 'decision']
        runner._authority(SimpleNamespace(data=authority('none')))  # released
        for _ in range(3):
            runner.tick()
        assert runner.started is None and not goals
        assert 'task_started' not in [e for e, _ in events]
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_a_goal_cancelled_by_someone_else_ends_the_run_after_the_tick(monkeypatch):
    rclpy.init()
    runner = None
    goals, events = [], []

    def send(route, precise=False, quiet=False, why=''):
        if runner.navigator.closed:
            return None
        goals.append(SimpleNamespace(done=False, status=None, cancel_requested=False,
                                     cancel=lambda: None))
        return goals[-1]

    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner.navigator, 'ready', lambda: True)
        monkeypatch.setattr(runner.navigator, 'send', send)
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 0.0, 0.5))
        monkeypatch.setattr(runner, 'report', lambda event, **d: events.append((event, d)))
        runner.tick()
        goals[0].done, goals[0].status = True, 'CANCELED'  # the TUI revoked it
        runner.tick()
        assert runner.outcome == 'PREEMPTED' and len(goals) == 1
        assert ('task_finished', {'outcome': 'PREEMPTED'}) in events
        for _ in range(3):
            runner.tick()
        assert len(goals) == 1
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_an_override_before_nav2_accepts_cancels_the_goal_on_the_way_out(monkeypatch):
    """Cancel a goal still in flight at an override on the real exit path.

    Once Nav2 accepts it, however busy high-rate topics keep the executor, it is
    cancelled and never left to drive.
    """
    import threading
    import time
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.executors import MultiThreadedExecutor
    from mhseals_nav.tasks import interface
    rclpy.init()
    runner = server = executor = None
    cancelled = []

    def server_for(kind):
        def execute(handle):
            deadline = time.monotonic() + 10.0
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                cancelled.append(kind.__name__)
                handle.canceled()
            else:
                handle.abort()  # left driving: what the fix prevents
            return kind.Result()
        return execute

    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
        runner.objects = [('yellow_buoy', 9.0, 0.0)]
        monkeypatch.setattr(runner, 'pose', lambda: (1.0, 0.0, 0.5))
        runner.navigator.backup_wait_s = 0.0  # this fake Nav2 has no backup server
        server = rclpy.create_node('fake_nav2')
        for kind, name in ((NavigateToPose, 'navigate_to_pose'),
                           (NavigateThroughPoses, 'navigate_through_poses')):
            ActionServer(server, kind, name, server_for(kind),
                         cancel_callback=lambda request: CancelResponse.ACCEPT)
        odom = server.create_publisher(Odometry, '/odom/local', 10)
        applied = server.create_publisher(Twist, '/cmd_vel/applied', 10)
        server.create_timer(0.001, lambda: (odom.publish(Odometry()), applied.publish(Twist())))
        deadline = time.monotonic() + 10.0  # Nav2 not spinning: the goal stays unaccepted
        while not runner.navigator.goals and time.monotonic() < deadline:
            rclpy.spin_once(runner, timeout_sec=0.05)
        goal = runner.navigator.goals[0]
        assert goal.handle is None
        runner._authority(SimpleNamespace(data=authority('rc')))
        assert runner.outcome == 'PREEMPTED'
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        threading.Thread(target=executor.spin, daemon=True).start()  # Nav2 wakes up, busy
        interface.run(runner)  # what main() does before the process exits
        assert goal.handle is not None and goal.handle.accepted and goal.settled
        deadline = time.monotonic() + 3.0
        while not cancelled and time.monotonic() < deadline:
            time.sleep(0.01)
        assert cancelled == ['NavigateToPose']
    finally:
        if executor is not None:
            executor.shutdown()
        for node in (server, runner):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()


def test_an_override_cancels_a_back_out_in_flight_and_none_goes_out_after(monkeypatch):
    """Track a BackUp goal like a navigation goal: cancel it on the way out."""
    import threading
    import time
    from nav2_msgs.action import BackUp
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.executors import MultiThreadedExecutor
    from mhseals_nav.tasks import interface
    rclpy.init()
    runner = server = executor = None
    cancelled = []

    def execute(handle):
        deadline = time.monotonic() + 10.0
        while not handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.01)
        if handle.is_cancel_requested:
            cancelled.append('BackUp')
            handle.canceled()
        else:
            handle.abort()
        return BackUp.Result()

    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
        monkeypatch.setattr(runner, 'pose', lambda: None)  # no pose: the tree sends nothing
        runner.started = runner.get_clock().now().nanoseconds / 1e9  # the task is under way
        server = rclpy.create_node('fake_behavior_server')
        ActionServer(server, BackUp, 'backup', execute,
                     cancel_callback=lambda request: CancelResponse.ACCEPT)
        assert runner.navigator.backup.wait_for_server(timeout_sec=10.0)
        reverse = (1.8, 0.1, 40.0)
        goal = runner.navigator.send([(0.0, 0.0, 0.0)], why='back out', reverse=reverse)
        assert goal is not None and goal.handle is None  # Nav2 not spinning: in flight
        runner._authority(SimpleNamespace(data=authority('rc')))
        assert runner.outcome == 'PREEMPTED'
        assert runner.navigator.send([(0.0, 0.0, 0.0)], reverse=reverse) is None
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        threading.Thread(target=executor.spin, daemon=True).start()
        interface.run(runner)
        assert goal.handle is not None and goal.handle.accepted and goal.settled
        deadline = time.monotonic() + 3.0
        while not cancelled and time.monotonic() < deadline:
            time.sleep(0.01)
        assert cancelled == ['BackUp']
    finally:
        if executor is not None:
            executor.shutdown()
        for node in (server, runner):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()


def test_ready_waits_for_the_backup_server_but_not_for_good(monkeypatch):
    from mhseals_nav.tasks import interface
    from mhseals_nav.tasks.interface import Navigator
    navigator = Navigator.__new__(Navigator)
    up = SimpleNamespace(server_is_ready=lambda: True)
    backup = [False]
    navigator.to_pose = navigator.through = up
    navigator.backup = SimpleNamespace(server_is_ready=lambda: backup[0])
    navigator.backup_wait_s, navigator.backup_since = 15.0, None
    warnings = []
    navigator.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda message, once=False: warnings.append(message)))
    clock = [100.0]
    monkeypatch.setattr(interface.time, 'monotonic', lambda: clock[0])
    assert not navigator.ready()
    backup[0] = True
    assert navigator.ready() and not warnings
    backup[0], clock[0] = False, 116.0  # never came up: start anyway, and say so
    assert navigator.ready() and len(warnings) == 1


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


def test_events_carry_their_schema_and_the_fixture_keys(monkeypatch):
    from pathlib import Path
    from mhseals_nav.tasks.interface import EVENTS_SCHEMA
    fixture = json.loads((Path(__file__).parent / 'fixtures/task_events.json').read_text())
    assert {e['schema'] for e in fixture} == {EVENTS_SCHEMA}
    decision = next(e for e in fixture if e['event'] == 'decision')
    rclpy.init()
    runner = None
    try:
        runner = TaskRunner('circle_test', dict(CIRCLE))
        published = []
        monkeypatch.setattr(runner.events, 'publish', lambda m: published.append(
            json.loads(m.data)))
        runner.stage = 'docking'
        runner.decide('start', 'starting docking')
        assert set(decision) <= set(published[-1])
        assert published[-1]['schema'] == EVENTS_SCHEMA
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_status_follows_the_mission_and_stale_tracks_expire(monkeypatch):
    """The status carries the fixture's keys; tracks the tracker stopped sending go."""
    from pathlib import Path
    from mhseals_nav.tasks.tracks import Track
    fixture = json.loads((Path(__file__).parent / 'fixtures/mission_status.json').read_text())
    rclpy.init()
    runner = None
    try:
        params = {'tasks': ['nav_channel', 'return_home'], 'timeout': 100.0}
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
        position = SimpleNamespace(x=20.0, y=5.0)
        runner._tracks(SimpleNamespace(tracks=[SimpleNamespace(  # a course-map prior
            class_id='green_pole_buoy', confidence=0.0, motion=1, source=1, id=9,
            pose=SimpleNamespace(covariance=[0.0] * 36, pose=SimpleNamespace(
                position=position)))]))
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
        assert [(t.label, t.prior) for t in runner.board.tracks] == [('green_pole_buoy', True)]
    finally:
        if runner is not None:
            runner.destroy_node()
        rclpy.try_shutdown()


def test_no_zero_twist_after_the_run_is_closed():
    """Navigator.hold sends nothing once an override (or the end) closed the run."""
    from mhseals_nav.tasks.interface import Navigator
    sent = []
    navigator = Navigator.__new__(Navigator)
    navigator.closed, navigator.goals = False, []
    navigator.zero = SimpleNamespace(publish=sent.append)
    navigator.hold()
    navigator.close()
    navigator.hold()
    assert len(sent) == 1


def test_settle_reports_goals_still_open(monkeypatch):
    """When exit_settle_s runs out with a goal open, it is warned and reported."""
    from mhseals_nav.tasks import interface
    events, warnings = [], []
    runner = SimpleNamespace(
        config={'exit_settle_s': 0.0},
        navigator=SimpleNamespace(settled=lambda: False, goals=[object()]),
        get_logger=lambda: SimpleNamespace(warning=warnings.append),
        report=lambda event, **d: events.append((event, d)))
    monkeypatch.setattr(interface.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(interface.rclpy, 'spin_once', lambda *a, **k: None)
    assert interface.TaskRunner.settle(runner) is False
    assert events == [('goals_unsettled', {'count': 1})] and warnings
