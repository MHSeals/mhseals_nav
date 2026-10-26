"""run_task: tick one task tree on ROS time and report what it does.

Interface the test suite scores and the TUI shows (all reliable, transient local):
  /task/events      std_msgs/String JSON, mhseals.task_events/1, one per event: {schema,
                    task, event, time, elapsed, ...}; ``decision`` events carry {kind,
                    reason} for the operator's log
  /task/status      std_msgs/String JSON, mhseals.mission_status/1: {task, status, elapsed,
                    tip, stage, phase, behaviour, goal, stages, remaining, recent}; on change
                    (at most 5 Hz) and at 1 Hz
  /task/hypotheses  std_msgs/String JSON, mhseals.task_hypotheses/1 (tasks/recognition.py)
  exit code         0 succeeded, 1 failed, 2 timed out, 3 stopped by a human override,
                    4 refused: a human had control when it was started

A human override (rc, keyboard, hold) ends the task at once, cancelling its Nav2 goals; it
never resumes by itself. Started while a human has control, the task refuses to start.
"""

import argparse
import json
import math
import sys
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3Stamped
from mavros_msgs.msg import GPSRAW
from mhseals_nav.tasks import load_config, tracks
from mhseals_nav.tasks.conditions import speed as conditions_speed
# config_path and load_params live in the ROS-free host; kept here for their callers.
from mhseals_nav.tasks.host import (config_path, EVENTS_SCHEMA, load_params,  # noqa: F401
                                    MissionHost)
from mhseals_nav.tasks.trees import TREES
from nav2_msgs.action import BackUp, NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

try:  # perception v2 messages (pv2-track); without them only /objects/map is read
    from mhseals_nav.msg import TrackArray
except ImportError:
    TrackArray = None

AUTHORITY_TOPIC = '/mhseals_testing/authority'


HUMAN_OWNERS = ('rc', 'keyboard', 'hold')


def claim_suspended(record):
    """Tell whether an authority record (JSON text) is owned by a human.

    The arbiter never publishes a task claim: a running task is only a source,
    so a human owner (rc, keyboard, hold) is what suspends it. A missing or
    unparsable record, or owner none/unknown, leaves the task free to run so
    it still works without an arbiter.
    """
    try:
        return json.loads(record).get('owner') in HUMAN_OWNERS
    except (ValueError, AttributeError, TypeError):
        return False


EXIT_CODES = {'SUCCESS': 0, 'FAILURE': 1, 'TIMEOUT': 2, 'PREEMPTED': 3, 'REFUSED': 4}


class Goal:
    """One Nav2 goal, polled without blocking."""

    def __init__(self, request):
        self.request, self.handle, self.result, self.cancelling = request, None, None, None
        self.direct = False  # drives /cmd_vel itself (BackUp), past the smoother
        self.cancel_requested = False
        request.add_done_callback(self._accepted)

    def _accepted(self, future):
        self.handle = future.result()
        if self.handle.accepted:
            self.result = self.handle.get_result_async()
            if self.cancel_requested:  # cancelled while the request was in flight
                self.cancelling = self.handle.cancel_goal_async()

    @property
    def settled(self):
        """Nav2 can no longer drive it: rejected, ended, or its cancel acknowledged."""
        if self.handle is None:
            return False
        return (not self.handle.accepted or self.result.done()
                or (self.cancelling is not None and self.cancelling.done()))

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
        if (self.handle is not None and self.handle.accepted and not self.done
                and self.cancelling is None):
            self.cancelling = self.handle.cancel_goal_async()


class Navigator:
    """Sends routes of (x, y, yaw) map poses to Nav2 without blocking the tick.

    Every goal sent stays tracked until Nav2 can no longer drive it, so ``close``
    can cancel them all, including one whose request is still in flight.
    """

    def __init__(self, node, frame='map', hold_topic='/cmd_vel_nav', backup_wait_s=0.0,
                 speed_limit_topic='/speed_limit'):
        from ament_index_python.packages import get_package_share_directory
        self.node, self.frame, self.report, self.last = node, frame, None, None
        self.backup_wait_s, self.backup_since = backup_wait_s, None
        self.goals, self.closed = [], False
        self.zero = node.create_publisher(Twist, hold_topic, 1)
        self.driven_at = None  # monotonic time of Nav2's last non-zero command on hold_topic
        node.create_subscription(Twist, hold_topic, self._driven, 10)
        self.limit = node.create_publisher(SpeedLimit, speed_limit_topic, 10)
        self.to_pose = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        self.backup = ActionClient(node, BackUp, 'backup')  # behavior_server's backup plugin
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
        """Command zero velocity through Nav2's smoother: holding still, with no goal on.

        Nothing goes out once the run is closed (an override, or the end): the task
        must not command the boat after a human took it.
        """
        if not self.closed:
            self.zero.publish(Twist())

    def _driven(self, message):
        v, w = message.linear, message.angular
        if any((v.x, v.y, v.z, w.x, w.y, w.z)):
            self.driven_at = time.monotonic()

    def keep_still(self, quiet_s=0.3):
        """Hold station with fresh zeros whenever Nav2 is not commanding the boat.

        Silence is not a hold: 0.5 s without a command the command_mux idles and the
        velocity controller gives neutral effort, so the boat drifts in the current.
        Nav2 is silent between goals, after an abort and through its recovery Wait.
        Nothing goes out while Nav2 drove within ``quiet_s`` (one publisher at a time),
        while a BackUp goal drives /cmd_vel itself, nor once an override or the end
        closed the run.
        """
        if self.closed or any(goal.direct and not goal.settled for goal in self.goals):
            return
        if self.driven_at is None or time.monotonic() - self.driven_at > quiet_s:
            self.hold()

    def speed_limit(self, speed):
        """Cap Nav2's controller at ``speed`` m/s; 0 lifts the cap.

        Lifting it still goes out after an override: it commands no motion, and a
        cap left behind would slow whoever drives with Nav2 next.
        """
        self.limit.publish(SpeedLimit(percentage=False, speed_limit=float(speed)))

    def ready(self):
        """Nav2's navigators are up, and the backup behaviour or ``backup_wait_s`` gone by.

        Without the backup server a dock exit drives out instead of reversing, so it is
        waited for, but only so long: no other task needs it.
        """
        if not (self.to_pose.server_is_ready() and self.through.server_is_ready()):
            return False
        if self.backup.server_is_ready():
            return True
        now = time.monotonic()
        self.backup_since = self.backup_since or now
        if now - self.backup_since < self.backup_wait_s:
            return False
        self.node.get_logger().warning(
            f'no backup server after {self.backup_wait_s} s: a dock exit will drive out',
            once=True)
        return True

    def close(self):
        """Cancel every goal sent; none is sent after this (None, as for a refusal)."""
        self.closed = True
        for goal in self.goals:
            goal.cancel()

    def settled(self):
        """Tell whether Nav2 holds no goal of this run that it could still drive."""
        self.goals = [goal for goal in self.goals if not goal.settled]
        return not self.goals

    def send(self, route, precise=False, quiet=False, why='', reverse=None):
        """Send a route; every send is a ``goal`` event unless ``quiet`` (a retry).

        ``reverse`` (distance m, speed m/s, allowed s) reverses straight towards the
        route's end with Nav2's BackUp instead of planning to it.
        """
        if self.closed or (reverse is not None and not self.backup.server_is_ready()):
            return None
        if route and not quiet:
            x, y, yaw = route[-1]
            self.last = {'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2), 'why': why}
            if self.report is not None:
                self.report('goal', **self.last)
        poses = [self.pose(*waypoint) for waypoint in route]
        if reverse is not None:
            distance, speed, seconds = reverse
            goal = Goal(self.backup.send_goal_async(BackUp.Goal(
                target=Point(x=float(distance)), speed=float(speed),
                time_allowance=Duration(sec=int(seconds)))))
        elif len(poses) == 1 or precise:
            request = NavigateToPose.Goal(pose=poses[-1])
            if precise:
                request.behavior_tree = self.precise_tree
            goal = Goal(self.to_pose.send_goal_async(request))
        else:
            goal = Goal(self.through.send_goal_async(NavigateThroughPoses.Goal(poses=poses)))
        goal.direct = reverse is not None
        self.settled()  # forget goals Nav2 is done with
        self.goals.append(goal)
        return goal


class TaskRunner(Node):
    """The ROS side of a run: topics, TF, Nav2 and the human override around MissionHost.

    Every per-tick decision is the host's (tasks/host.py); this node feeds it the pose,
    tracks and field of view, publishes what it reports, and ends it on an override.
    """

    def __init__(self, task, params):
        super().__init__(f'task_{task}')
        self.task = task
        self.config = load_config('runner', params.get('runner'))
        self.frame = params.get('map_frame', 'map')
        self.v2_at = None  # last /perception/tracks message: it supersedes /objects/map
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Kept for late joiners: a run's first events go out before a watcher that
        # started with it has matched the publisher.
        self.events = self.create_publisher(
            String, '/task/events',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(String, '/task/status', latched)
        self.hypotheses = self.create_publisher(String, '/task/hypotheses', latched)
        navigator = Navigator(self, self.frame, params.get('hold_topic', '/cmd_vel_nav'),
                              float(self.config['backup_wait_s']))
        self.host = MissionHost(
            task, params, navigator, self._publish_event, lambda: self.now(),
            status_sink=lambda status: self.status.publish(
                String(data=json.dumps(status, default=str))),
            hypotheses_sink=lambda message: self.hypotheses.publish(
                String(data=json.dumps(message))),
            on_tick=self._tracking)
        self.create_subscription(Detection3DArray, params.get('objects_topic', '/objects/map'),
                                 self._objects, 10)
        if TrackArray is not None:
            self.create_subscription(TrackArray, params.get('tracks_topic', '/perception/tracks'),
                                     self._tracks, 10)
        self.create_subscription(DiagnosticArray, '/perception/status', self._status, 10)
        self.watch_conditions(self.host.conditions)
        # A human override (rc, keyboard, hold) ends the task the moment it arrives.
        self.suspended = False
        self.create_subscription(String, AUTHORITY_TOPIC, self._authority, latched)
        self.create_timer(1.0 / float(params.get('tick_hz', 10.0)), self.tick)

    def _publish_event(self, payload):
        self.events.publish(String(data=json.dumps(payload, default=str)))
        details = {k: v for k, v in payload.items()
                   if k not in ('schema', 'task', 'event', 'time', 'elapsed')}
        self.get_logger().info(f"{payload['event']} {details}")

    def _objects(self, message):
        # Remembered for a few seconds, so a track that drops out briefly does not
        # make a sighting flicker. Superseded while perception v2 tracks arrive.
        if self.v2_at is None or self.now() - self.v2_at > self.config['tracks_supersede_s']:
            self.host.remember(tracks.from_detections(message), priors=False)

    def _tracks(self, message):
        self.v2_at = self.now()
        self.host.remember(tracks.from_track_array(message))  # as MissionHost.tick does

    def _status(self, message):
        """Take the camera's field of view from the perception backend's status."""
        for status in message.status:
            values = {kv.key: kv.value for kv in status.values}
            try:
                if 'hfov_deg' in values and 'range_m' in values:
                    self.host.fov = (math.radians(float(values['hfov_deg'])),
                                     float(values['range_m']))
            except ValueError:
                pass

    def watch_conditions(self, conditions):
        """Feed tasks/conditions.py from small live topics; an empty topic disables one.

        GPS_RAW_INT and the drift estimate arrive at a few Hz. The velocity check keeps
        only the latest applied command, odometry twist and effort, judged at the tick.
        """
        topics = conditions.cfg
        self.velocity = {'command': None, 'measured': None, 'effort': None}
        if topics['fix_topic']:
            self.create_subscription(GPSRAW, topics['fix_topic'],
                                     lambda m: setattr(conditions, 'fix', m.fix_type), 1)
        if topics['drift_topic']:
            self.create_subscription(
                Vector3Stamped, topics['drift_topic'],
                lambda m: conditions.drift(conditions_speed(m.vector), self.now()), 1)
        if topics['command_topic'] and topics['odom_topic'] and topics['effort_topic']:
            def keep(key, value):
                self.velocity[key] = value
            self.create_subscription(Twist, topics['command_topic'], lambda m: keep(
                'command', (m.linear.x, m.angular.z)), 1)
            self.create_subscription(Odometry, topics['odom_topic'], lambda m: keep(
                'measured', (m.twist.twist.linear.x, m.twist.twist.angular.z)), 1)
            self.create_subscription(Twist, topics['effort_topic'], lambda m: keep(
                'effort', (m.linear.x, m.linear.y, m.angular.z)), 1)
        return conditions

    def _tracking(self, now):
        if None not in self.velocity.values():
            self.host.conditions.tracking(self.velocity['command'], self.velocity['measured'],
                                          self.velocity['effort'], now)

    def _authority(self, message):
        try:
            self.host.owner = json.loads(message.data).get('owner')
        except (ValueError, AttributeError, TypeError):
            self.host.owner = None
        self.suspended = claim_suspended(message.data)
        if self.suspended and self.host.outcome is None and self.host.started is not None:
            self.preempt()
        elif self.suspended and self.host.outcome is None:
            self.refuse()

    def refuse(self):
        self.host.refuse()

    def preempt(self, why=None):
        self.host.preempt(why)

    def decide(self, kind, reason, **details):
        self.host.decide(kind, reason, **details)

    def report(self, event, **details):
        self.host.report(event, **details)

    def finish(self, outcome):
        self.host.finish(outcome)

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

    def tick(self):
        if self.host.outcome is not None:
            return
        if self.host.started is None and self.now() == 0:  # waiting for /clock
            return
        self.host.tick(self.pose())

    def settle(self):
        """Spin until Nav2 holds no goal of this run, at most ``exit_settle_s`` (wall).

        A goal still in flight is cancelled once Nav2 accepts it, so the process must
        live until then: else Nav2 drives an orphan as soon as the mux returns to auto.
        """
        deadline = time.monotonic() + float(self.config['exit_settle_s'])
        while rclpy.ok() and not self.host.navigator.settled() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        settled = self.host.navigator.settled()
        if not settled:  # an operator must know a goal may still be live in Nav2
            open_goals = len(self.host.navigator.goals)
            self.get_logger().warning(f'{open_goals} Nav2 goal(s) still open after '
                                      f"{self.config['exit_settle_s']} s")
            self.report('goals_unsettled', count=open_goals)
        for _ in range(5):  # let the final events and status go out
            rclpy.spin_once(self, timeout_sec=0.05)
        return settled


def run(runner):
    """Tick until the task ends, then stay until Nav2 holds none of its goals."""
    try:
        while rclpy.ok() and runner.host.outcome is None:
            rclpy.spin_once(runner, timeout_sec=0.1)
    except KeyboardInterrupt:
        runner.finish('FAILURE')
    try:
        runner.settle()
    except KeyboardInterrupt:  # a second Ctrl+C: leave now
        pass


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
        run(runner)
    finally:
        runner.destroy_node()
        rclpy.try_shutdown()
    sys.exit(EXIT_CODES[runner.host.outcome or 'FAILURE'])


if __name__ == '__main__':
    main()
