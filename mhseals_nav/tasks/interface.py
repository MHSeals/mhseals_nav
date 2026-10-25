"""run_task: tick one task tree on ROS time and report what it does.

Interface the test suite scores and the TUI shows (all reliable, transient local):
  /task/events      std_msgs/String JSON, one per event: {task, event, time, elapsed, ...};
                    ``decision`` events carry {kind, reason} for the operator's log
  /task/status      std_msgs/String JSON, mhseals.mission_status/1: {task, status, elapsed,
                    tip, stage, phase, behaviour, goal, stages, remaining, recent}; on change
                    (at most 5 Hz) and at 1 Hz
  /task/hypotheses  std_msgs/String JSON, mhseals.task_hypotheses/1 (tasks/recognition.py)
  exit code         0 succeeded, 1 failed, 2 timed out, 3 stopped by a human override

A human override (rc, keyboard, hold) ends the task at once, cancelling its Nav2 goal; it
never resumes by itself. Before the task starts, a human owner only delays the start.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from diagnostic_msgs.msg import DiagnosticArray

from mhseals_nav.tasks import tracks
from mhseals_nav.tasks.trees import KINDS, TREES

try:  # perception v2 messages (pv2-track); without them only /objects/map is read
    from mhseals_nav.msg import TrackArray
except ImportError:
    TrackArray = None
from mhseals_nav.tasks.search import Coverage, load_config as load_search
from mhseals_nav.tasks.recognition import Recogniser

AUTHORITY_TOPIC = '/mhseals_testing/authority'


HUMAN_OWNERS = ('rc', 'keyboard', 'hold')


def claim_suspended(record):
    """True when an authority record (JSON text) is owned by a human.

    The arbiter never publishes a task claim: a running task is only a source,
    so a human owner (rc, keyboard, hold) is what suspends it. A missing or
    unparsable record, or owner none/unknown, leaves the task free to run so
    it still works without an arbiter.
    """
    try:
        return json.loads(record).get('owner') in HUMAN_OWNERS
    except (ValueError, AttributeError, TypeError):
        return False


EXIT_CODES = {'SUCCESS': 0, 'FAILURE': 1, 'TIMEOUT': 2, 'PREEMPTED': 3}


class Goal:
    """One Nav2 goal, polled without blocking."""

    def __init__(self, request):
        self.request, self.handle, self.result = request, None, None
        self.cancel_requested = False
        request.add_done_callback(self._accepted)

    def _accepted(self, future):
        self.handle = future.result()
        if self.handle.accepted:
            self.result = self.handle.get_result_async()
            if self.cancel_requested:  # cancelled while the request was in flight
                self.handle.cancel_goal_async()

    @property
    def done(self):
        if self.handle is not None and not self.handle.accepted:
            return True
        return self.result is not None and self.result.done()

    @property
    def status(self):
        if self.handle is not None and not self.handle.accepted:
            return 'REJECTED'
        return {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(self.result.result().status, 'UNKNOWN')

    def cancel(self):
        self.cancel_requested = True
        if self.handle is not None and self.handle.accepted and not self.done:
            self.handle.cancel_goal_async()


class Navigator:
    """Sends routes of (x, y, yaw) map poses to Nav2 without blocking the tick."""

    def __init__(self, node, frame='map', hold_topic='/cmd_vel_nav'):
        from ament_index_python.packages import get_package_share_directory
        self.node, self.frame, self.report, self.last = node, frame, None, None
        self.zero = node.create_publisher(Twist, hold_topic, 1)
        self.to_pose = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        # Docking ends on the tight goal checker (behavior_trees/nav_precise.xml).
        self.precise_tree = (f"{get_package_share_directory('mhseals_nav')}"
                             '/behavior_trees/nav_precise.xml')

    def pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return pose

    def hold(self):
        """Command zero velocity through Nav2's smoother: holding still, with no goal on."""
        self.zero.publish(Twist())

    def ready(self):
        return self.to_pose.server_is_ready() and self.through.server_is_ready()

    def send(self, route, precise=False, quiet=False, why=''):
        """Send a route; every send is a ``goal`` event unless ``quiet`` (a retry)."""
        if route and not quiet:
            x, y, yaw = route[-1]
            self.last = dict(x=round(x, 2), y=round(y, 2), yaw=round(yaw, 2), why=why)
            if self.report is not None:
                self.report('goal', **self.last)
        poses = [self.pose(*waypoint) for waypoint in route]
        if len(poses) == 1 or precise:
            goal = NavigateToPose.Goal(pose=poses[-1])
            if precise:
                goal.behavior_tree = self.precise_tree
            return Goal(self.to_pose.send_goal_async(goal))
        return Goal(self.through.send_goal_async(NavigateThroughPoses.Goal(poses=poses)))


class TaskRunner(Node):
    STATUS_PERIOD, STATUS_MIN_GAP, RECENT = 1.0, 0.2, 5  # s, s, decisions kept in the status

    def __init__(self, task, params):
        super().__init__(f'task_{task}')
        self.task = task
        self.timeout = float(params.get('timeout', 600.0))
        self.frame = params.get('map_frame', 'map')
        self.objects, self.tracks, self.memory = [], [], []
        self.memory_horizon = float(params.get('memory', 5.0))
        self.v2_at = None  # last /perception/tracks message: it supersedes /objects/map
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Detection3DArray, params.get('objects_topic', '/objects/map'),
                                 self._objects, 10)
        if TrackArray is not None:
            self.create_subscription(TrackArray, params.get('tracks_topic', '/perception/tracks'),
                                     self._tracks, 10)
        search = load_search(params.get('search'))
        self.fov = (math.radians(float(search['hfov_deg'])), float(search['range_m']))
        self.create_subscription(DiagnosticArray, '/perception/status', self._status, 10)
        self.coverage_config = search
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Kept for late joiners: a run's first events go out before a watcher that
        # started with it has matched the publisher.
        self.events = self.create_publisher(
            String, '/task/events',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(String, '/task/status', latched)
        self.hypotheses = self.create_publisher(String, '/task/hypotheses', latched)
        self.recogniser = Recogniser.from_yaml(config_path('recognition.yaml'))
        self.status_at, self.status_last, self.hypotheses_at = None, None, None
        stages = params.get('tasks', [task] if task != 'mission' else [])
        self.stages = {name: dict(state='pending', attempts=0, failures=0, elapsed=0.0,
                                  since=None) for name in stages}
        self.stage, self.recent = None, []
        # A human override (rc, keyboard, hold) ends the task the moment it arrives.
        self.suspended, self.owner = False, None
        self.create_subscription(String, AUTHORITY_TOPIC, self._authority, latched)
        self.board = py_trees.blackboard.Client(name='runner')
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage', 'hypotheses'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.board.home = self.board.coverage = None
        self.board.fov = self.fov
        self.tree = py_trees.trees.BehaviourTree(TREES[task](params))
        self.navigator = Navigator(self, self.frame, params.get('hold_topic', '/cmd_vel_nav'))
        self.navigator.report = self.report
        self.tree.setup(timeout=15.0, navigator=self.navigator, report=self.report)
        self.started = None
        self.outcome = None
        self.create_timer(1.0 / float(params.get('tick_hz', 10.0)), self.tick)

    def _remember(self, seen):
        self.memory = tracks.remember(self.memory, seen, self.now(), self.memory_horizon)
        self._world(seen)

    def _world(self, priors=()):
        remembered = [track for track, _ in self.memory]
        self.tracks = remembered + [t for t in priors if t.prior]
        self.objects = tracks.objects(remembered)

    def _objects(self, message):
        # Remembered for a few seconds, so a track that drops out briefly does not
        # make a sighting flicker. Superseded while perception v2 tracks arrive.
        if self.v2_at is None or self.now() - self.v2_at > 2.0:
            self._remember(tracks.from_detections(message))

    def _tracks(self, message):
        self.v2_at = self.now()
        self._remember(tracks.from_track_array(message))

    def _status(self, message):
        """Take the camera's field of view from the perception backend's status."""
        for status in message.status:
            values = {kv.key: kv.value for kv in status.values}
            try:
                if 'hfov_deg' in values and 'range_m' in values:
                    self.fov = (math.radians(float(values['hfov_deg'])),
                                float(values['range_m']))
            except ValueError:
                pass

    def _authority(self, message):
        try:
            self.owner = json.loads(message.data).get('owner')
        except (ValueError, AttributeError, TypeError):
            self.owner = None
        self.suspended = claim_suspended(message.data)
        if self.suspended and self.started is not None and self.outcome is None:
            self.preempt()

    def preempt(self):
        """Yield to the human at once: cancel the goal and end; never resume."""
        self.decide('give_up', f'{self.owner or "human"} override: stopped, no resume')
        self.report('task_suspended', owner=self.owner)
        self.finish('PREEMPTED')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.frame, 'base_link', Time()).transform
        except TransformException:
            return None
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def decide(self, kind, reason, **details):
        self.report('decision', kind=kind, reason=reason[:60], **details)

    def report(self, event, **details):
        now = self.now()
        if event == 'decision' and self.stage and 'stage' not in details:
            details['stage'] = self.stage
        payload = dict(task=self.task, event=event, time=now,
                       elapsed=now - self.started if self.started is not None else 0.0, **details)
        self.events.publish(String(data=json.dumps(payload, default=str)))
        self.get_logger().info(f'{event} {details}')
        self.follow(event, details, payload)

    def follow(self, event, details, payload):
        """Keep the mission's stage table from the events the trees report."""
        now, stage = payload['time'], self.stages.get(details.get('stage'))
        if event == 'decision':
            self.recent = (self.recent + [payload])[-self.RECENT:]
        elif event == 'stage_started' and stage is not None:
            self.stage = details['stage']
            stage.update(state='running', since=now)
            stage['attempts'] += 1
        elif event in ('stage_completed', 'stage_skipped', 'stage_failed') and stage:
            kind = KINDS.get(details['stage'])
            if kind and event != 'stage_failed':
                self.recogniser.mark(kind, 'done' if event == 'stage_completed' else 'failed')
            stage['state'] = dict(stage_completed='done', stage_skipped='skipped',
                                  stage_failed='running')[event]
            if event == 'stage_failed':
                stage['failures'] += 1
            if stage['since'] is not None:
                stage['elapsed'] = round(now - stage['since'], 1)
            if event != 'stage_failed':
                self.stage = None
        else:
            return
        self.status_at = None  # publish on change

    def tick(self):
        if self.outcome is not None:
            return
        now = self.now()
        if self.suspended:  # a human holds the boat: the task has not started yet
            return
        pose = self.pose()
        if self.started is None:
            if now == 0:  # waiting for /clock
                return
            # A goal sent before discovery has matched Nav2's action servers is lost,
            # and its future never completes: the tree would wait on it for good.
            # Action discovery can finish before this process receives TF.
            # Planning also needs the initial map pose (and a valid home).
            if pose is None or not self.navigator.ready():
                return
            self.started = now
            self.report('task_started')
        # Forget tracks the tracker stopped sending: it may have died.
        if self.memory:
            before = len(self.memory)
            self.memory = tracks.remember(self.memory, [], now, self.memory_horizon)
            if len(self.memory) != before:
                self._world()
        self.board.objects, self.board.tracks = self.objects, self.tracks
        self.board.pose, self.board.now, self.board.fov = pose, now, self.fov
        if self.board.home is None:
            self.board.home = self.board.pose  # where the run started, for the return
            self.report('home', x=round(pose[0], 2), y=round(pose[1], 2))
            c = self.coverage_config
            self.board.coverage = Coverage(c['area'], c['cell'], pose, c['radius'],
                                           c['forget_s'])
        if self.board.coverage is not None:
            self.board.coverage.observe(pose, self.fov, now)
        self.recognise(now)
        self.tree.tick()
        status = self.tree.root.status.name
        if status in ('SUCCESS', 'FAILURE'):
            self.finish(status)
        elif now - self.started > self.timeout:
            self.finish('TIMEOUT')
        else:
            self.publish_status(status)

    def recognise(self, now):
        if self.recogniser is None:
            return
        self.board.hypotheses = self.recogniser.update(self.tracks, self.board.home, now)
        if self.hypotheses_at is None or now - self.hypotheses_at >= self.STATUS_PERIOD:
            self.hypotheses_at = now
            self.hypotheses.publish(String(data=json.dumps(self.recogniser.message(now))))

    def publish_status(self, status, force=False):
        now = self.now()
        if not force and self.status_at is not None and (
                now - self.status_at < self.STATUS_PERIOD):
            return
        if not force and self.status_last is not None and now - self.status_last < (
                self.STATUS_MIN_GAP):
            return
        self.status_at = self.status_last = now
        tip = self.tree.root.tip()
        stages = {name: {k: v for k, v in stage.items() if k != 'since'}
                  for name, stage in self.stages.items()}
        for name, stage in self.stages.items():
            if stage['state'] == 'running' and stage['since'] is not None:
                stages[name]['elapsed'] = round(now - stage['since'], 1)
        self.status.publish(String(data=json.dumps(dict(
            schema='mhseals.mission_status/1', task=self.task, status=self.outcome or status,
            elapsed=round(now - (self.started or now), 1), tip=tip.name if tip else None,
            stage=self.stage or (self.task if self.task != 'mission' else None),
            phase=getattr(tip, 'phase', None) if self.outcome is None else 'done',
            behaviour=tip.name if tip else None, goal=self.navigator.last, stages=stages,
            remaining=[n for n, st in self.stages.items() if st['state'] == 'pending'],
            recent=self.recent), default=str)))

    def finish(self, outcome):
        self.outcome = outcome
        self.tree.root.stop(py_trees.common.Status.INVALID)  # cancels a running Nav2 goal
        self.report('task_finished', outcome=outcome)
        if self.started is not None:
            self.publish_status(outcome, force=True)


def config_path(name):
    """A task YAML from the installed share, else from this source tree."""
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        path = Path(get_package_share_directory('mhseals_nav')) / 'config/tasks' / name
    except PackageNotFoundError:
        path = None
    if path is None or not path.exists():
        path = Path(__file__).resolve().parents[2] / 'config/tasks' / name
    return str(path)


def load_params(task, path=None):
    """Load a task's YAML; for the mission, each task's own YAML under its name."""
    with open(path or config_path(f'{task}.yaml'), encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}
    if task == 'mission':
        for name in params['tasks']:
            params[name] = {**load_params(name), **params.get(name, {})}
    return params


def main(args=None):
    parser = argparse.ArgumentParser(description='Run one RoboBoat task tree')
    parser.add_argument('--task', required=True, choices=sorted(TREES))
    parser.add_argument('--params', help='task YAML (default: config/tasks/<task>.yaml)')
    argv = rclpy.utilities.remove_ros_args(args=sys.argv if args is None else args)
    options = parser.parse_args(argv[1:])
    # Without rclpy's SIGINT handler, Ctrl+C reaches finish() with a live context, so
    # the final event goes out and a running Nav2 goal is cancelled.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    runner = TaskRunner(options.task, load_params(options.task, options.params))
    try:
        while rclpy.ok() and runner.outcome is None:
            rclpy.spin_once(runner, timeout_sec=0.1)
        for _ in range(5):  # let the final events and status go out
            rclpy.spin_once(runner, timeout_sec=0.05)
    except KeyboardInterrupt:
        runner.finish('FAILURE')
    finally:
        runner.destroy_node()
        rclpy.try_shutdown()
    sys.exit(EXIT_CODES[runner.outcome or 'FAILURE'])


if __name__ == '__main__':
    main()
