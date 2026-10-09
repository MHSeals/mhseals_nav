"""Isolated Nav2 commissioning smoke test, NOT a boat dynamics simulation.

Run only in a network-isolated container with ROS_DOMAIN_ID=231, ROS sourced.
Uses synthetic clear-water point returns and ideal velocity tracking.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from lifecycle_msgs.srv import GetState
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231', 'Isolate the synthetic test'
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/nav2_params.yaml').read_text())
    bt = config['bt_navigator']['ros__parameters']
    for key, filename in [('default_nav_to_pose_bt_xml', 'nav_to_pose.xml'),
                          ('default_nav_through_poses_bt_xml', 'nav_replan_recov.xml')]:
        bt[key] = str(root / 'mhseals_nav/behavior_trees' / filename)
    rclpy.init()
    node = rclpy.create_node('nav2_isolated_probe')
    odom_pub = node.create_publisher(Odometry, '/odom/local', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/points', 10)
    tf = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    origin = TransformStamped()
    origin.header.frame_id, origin.child_frame_id = 'map', 'odom'
    origin.transform.rotation.w = 1.0
    static.sendTransform(origin)
    state = [0.0, 0.0, 0.0]
    command = Twist()
    received = []
    lidar_required = '--no-lidar' not in sys.argv
    lidar_enabled = lidar_required

    def on_command(msg):
        nonlocal command
        command = msg
        received.append((time.monotonic(), msg.linear.x, msg.angular.z))
        assert -0.001 <= msg.linear.x <= 0.401, msg
        assert abs(msg.linear.y) < 0.001 and abs(msg.angular.z) <= 0.401, msg

    subscription = node.create_subscription(Twist, '/cmd_vel', on_command, 10)

    def tick():
        state[2] += command.angular.z * 0.05
        state[0] += command.linear.x * math.cos(state[2]) * 0.05
        state[1] += command.linear.x * math.sin(state[2]) * 0.05
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = state[:2]
        transform.transform.rotation.z = math.sin(state[2] / 2)
        transform.transform.rotation.w = math.cos(state[2] / 2)
        tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state[:2]
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist = command
        odom_pub.publish(odom)
        if lidar_enabled:
            header = Header(stamp=stamp, frame_id='base_link')
            points = [(18 * math.cos(i * math.pi / 36),
                       18 * math.sin(i * math.pi / 36), 0.3) for i in range(72)]
            cloud_pub.publish(create_cloud_xyz32(header, points))

    timer = node.create_timer(0.05, tick)
    process = None

    def spin_until(predicate, timeout):
        start = time.monotonic()
        while not predicate() and time.monotonic() - start < timeout:
            assert process is None or process.poll() is None, 'Navigation launch exited'
            rclpy.spin_once(node, timeout_sec=0.05)
        assert predicate(), f'Timed out after {timeout}s'

    def pose(x):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = node.get_clock().now().to_msg()
        result.pose.position.x = x
        result.pose.orientation.w = 1.0
        return result

    with tempfile.TemporaryDirectory(prefix='nav2-probe-') as directory:
        params = Path(directory) / 'params.yaml'
        params.write_text(yaml.safe_dump(config))
        with (Path(directory) / 'nav2.log').open('w+') as log:
            launch_command = [
                'ros2', 'launch', str(root / 'launch/navigation.launch.py'),
                'params_file:=' + str(params), 'sim:=false']
            if '--installed-launch' in sys.argv:
                launch_command = ['ros2', 'launch', 'mhseals_nav', 'navigation.launch.py',
                                  'params_file:=' + str(params), 'sim:=false']
            launch_command.append('use_lidar:=' + str(lidar_required).lower())
            process = subprocess.Popen(launch_command,
                stdout=log, stderr=log, start_new_session=True)
            try:
                client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
                spin_until(client.server_is_ready, 40)
                lifecycle = node.create_client(GetState, '/velocity_smoother/get_state')
                spin_until(lifecycle.service_is_ready, 10)
                deadline = time.monotonic() + 30
                while True:
                    active = lifecycle.call_async(GetState.Request())
                    spin_until(active.done, 5)
                    if active.result().current_state.id == 3:
                        break
                    assert time.monotonic() < deadline, 'Lifecycle activation failed'
                    pause = time.monotonic()
                    spin_until(lambda: time.monotonic() - pause > 0.2, 1)
                goal = NavigateToPose.Goal()
                goal.pose = pose(3.0)
                future = client.send_goal_async(goal)
                spin_until(future.done, 10)
                handle = future.result()
                assert handle.accepted
                result = handle.get_result_async()
                spin_until(result.done, 50)
                assert result.result().status == 4, result.result()
                assert state[0] > 1.5, state
                assert any(v > 0.05 for _, v, _ in received)
                print('PASS NavigateToPose + controller + smoother, pose:', state, flush=True)
                through = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
                spin_until(through.server_is_ready, 10)
                goal2 = NavigateThroughPoses.Goal()
                goal2.poses = [pose(state[0] + 5.0)]
                future = through.send_goal_async(goal2)
                spin_until(future.done, 10)
                handle2 = future.result()
                assert handle2.accepted
                start = time.monotonic()
                spin_until(lambda: any(t > start and v > 0.05 for t, v, _ in received), 10)
                lidar_enabled = False
                loss = time.monotonic()
                spin_until(lambda: time.monotonic() - loss > 4.0, 6)
                recent = [(v, w) for t, v, w in received if t > loss + 3.0]
                # The smoother publishes a terminal zero then stops publishing.
                if lidar_required:
                    assert any(t > loss and abs(v) < 0.001 and abs(w) < 0.001
                               for t, v, w in received), 'No terminal stop command'
                    assert abs(command.linear.x) < 0.001 and abs(command.angular.z) < 0.001
                    assert all(abs(v) < 0.001 and abs(w) < 0.001 for v, w in recent), recent
                    print('PASS NavigateThroughPoses + stale lidar stops commands', flush=True)
                else:
                    assert any(v > 0.05 for v, _ in recent), recent
                    print('PASS explicit no-lidar mode navigates without /points', flush=True)
                handle2.cancel_goal_async()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                log.seek(0)
                print(log.read()[-16000:])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
