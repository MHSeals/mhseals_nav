"""run_task: tick one task tree on ROS time and report what it does.

Interface the test suite scores (and anyone can watch):
  /task/events  std_msgs/String JSON, one per event: {task, event, time, elapsed, ...}
  /task/status  std_msgs/String JSON, latched: {task, status, elapsed, tip}
  exit code     0 succeeded, 1 failed, 2 timed out
"""

import argparse
import json
import math
import sys

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from mhseals_nav.tasks import geometry
from mhseals_nav.tasks.trees import TREES

EXIT_CODES = {'SUCCESS': 0, 'FAILURE': 1, 'TIMEOUT': 2}


class Goal:
    def __init__(self, request):
        self.request, self.handle, self.result = request, None, None
        request.add_done_callback(self._accepted)

    def _accepted(self, future):
        self.handle = future.result()
        if self.handle.accepted:
            self.result = self.handle.get_result_async()

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
        if self.handle is not None and self.handle.accepted and not self.done:
            self.handle.cancel_goal_async()


class Navigator:
    """Sends routes of (x, y, yaw) map poses to Nav2 without blocking the tick."""

    def __init__(self, node, frame='map'):
        from ament_index_python.packages import get_package_share_directory
        self.node, self.frame = node, frame
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

    def ready(self):
        return self.to_pose.server_is_ready() and self.through.server_is_ready()

    def send(self, route, precise=False):
        poses = [self.pose(*waypoint) for waypoint in route]
        if len(poses) == 1 or precise:
            goal = NavigateToPose.Goal(pose=poses[-1])
            if precise:
                goal.behavior_tree = self.precise_tree
            return Goal(self.to_pose.send_goal_async(goal))
        return Goal(self.through.send_goal_async(NavigateThroughPoses.Goal(poses=poses)))


class TaskRunner(Node):
    def __init__(self, task, params):
        super().__init__(f'task_{task}')
        self.task = task
        self.timeout = float(params.get('timeout', 600.0))
        self.frame = params.get('map_frame', 'map')
        self.objects, self.memory = [], []
        self.memory_horizon = float(params.get('memory', 5.0))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Detection3DArray, params.get('objects_topic', '/objects/map'),
                                 self._objects, 10)
        # Kept for late joiners: a run's first events go out before a watcher that
        # started with it has matched the publisher.
        self.events = self.create_publisher(
            String, '/task/events',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(
            String, '/task/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.board = py_trees.blackboard.Client(name='runner')
        for key in ('objects', 'pose', 'now', 'home'):
            self.board.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.board.home = None
        self.tree = py_trees.trees.BehaviourTree(TREES[task](params))
        self.navigator = Navigator(self, self.frame)
        self.tree.setup(timeout=15.0, navigator=self.navigator, report=self.report)
        self.started = None
        self.outcome = None
        self.create_timer(1.0 / float(params.get('tick_hz', 10.0)), self.tick)

    def _objects(self, message):
        # Remembered for a few seconds, so a track that drops out briefly does not
        # make a sighting flicker.
        seen = [(d.results[0].hypothesis.class_id, d.bbox.center.position.x,
                 d.bbox.center.position.y) for d in message.detections if d.results]
        self.memory = geometry.remember(self.memory, seen, self.now(), self.memory_horizon)
        self.objects = [item[:3] for item in self.memory]

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

    def report(self, event, **details):
        now = self.now()
        payload = dict(task=self.task, event=event, time=now,
                       elapsed=now - self.started if self.started is not None else 0.0, **details)
        self.events.publish(String(data=json.dumps(payload, default=str)))
        self.get_logger().info(f'{event} {details}')

    def tick(self):
        if self.outcome is not None:
            return
        now = self.now()
        if self.started is None:
            if now == 0:  # waiting for /clock
                return
            # A goal sent before discovery has matched Nav2's action servers is lost,
            # and its future never completes: the tree would wait on it for good.
            if not self.navigator.ready():
                return
            self.started = now
            self.report('task_started')
        self.board.objects, self.board.pose, self.board.now = self.objects, self.pose(), now
        if self.board.home is None:
            self.board.home = self.board.pose  # where the run started, for the return
        self.tree.tick()
        status = self.tree.root.status.name
        if status in ('SUCCESS', 'FAILURE'):
            self.finish(status)
        elif now - self.started > self.timeout:
            self.finish('TIMEOUT')
        self.publish_status(status)

    def publish_status(self, status):
        tip = self.tree.root.tip()
        self.status.publish(String(data=json.dumps(dict(
            task=self.task, status=self.outcome or status,
            elapsed=self.now() - self.started, tip=tip.name if tip else None))))

    def finish(self, outcome):
        self.outcome = outcome
        self.tree.root.stop(py_trees.common.Status.INVALID)  # cancels a running Nav2 goal
        self.report('task_finished', outcome=outcome)


def load_params(task, path=None):
    """Load a task's YAML; for the mission, each task's own YAML under its name."""
    from ament_index_python.packages import get_package_share_directory
    folder = f"{get_package_share_directory('mhseals_nav')}/config/tasks"
    with open(path or f'{folder}/{task}.yaml', encoding='utf-8') as stream:
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
