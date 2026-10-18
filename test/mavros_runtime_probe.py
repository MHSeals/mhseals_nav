"""Isolated hardware-input integration test; never connects to the FCU or commands motion."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

if os.environ.get('ROS_DOMAIN_ID') != '231':
    raise SystemExit('Set ROS_DOMAIN_ID=231 for this synthetic test')
rclpy.init()
n = rclpy.create_node('mavros_contract_probe')
seen = []
n.create_subscription(Odometry, '/odom/local', lambda m: seen.append(m), 10)
raw = n.create_publisher(Odometry, '/mavros/global_position/local', 10)
imu = n.create_publisher(Imu, '/imu/raw', 10)
yaw = math.radians(-105)  # boat heading 195 degrees in ENU
processes = []
try:
    processes.append(subprocess.Popen([sys.executable, '-c',
        'from mhseals_nav.mavros_odometry import main; main()']))
    binary = Path(get_package_prefix('robot_localization')) / 'lib/robot_localization/ekf_node'
    config = Path(__file__).resolve().parents[1] / 'config/ekf_local.yaml'
    processes.append(subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config),
        '-r', '__node:=ekf_local', '-r', 'odometry/filtered:=/odom/local']))
    def send():
        stamp = n.get_clock().now().to_msg()
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id, m.child_frame_id = 'map', 'base_link'
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        m.twist.twist.linear.x, m.twist.twist.linear.y = math.cos(yaw), math.sin(yaw)
        m.twist.covariance[0] = -1.
        raw.publish(m)
        i = Imu()
        i.header.stamp, i.header.frame_id = stamp, 'base_link'
        i.orientation = m.pose.pose.orientation
        i.orientation_covariance[8] = i.angular_velocity_covariance[8] = .01
        imu.publish(i)
    timer = n.create_timer(.05, send)
    start = time.monotonic()
    while time.monotonic()-start < 6:
        rclpy.spin_once(n, timeout_sec=.02)
    assert len(seen) > 80, f'Only {len(seen)} EKF samples'
    v = seen[-1].twist.twist.linear
    assert abs(v.x - 1.) < .03 and abs(v.y) < .03, (v.x, v.y)
    print(f'heading 195 degrees: EKF surge={v.x:.3f}, sway={v.y:.3f}; {len(seen)} samples')
finally:
    for p in processes:
        p.terminate()
    for p in processes:
        p.wait(timeout=5)
    assert processes[0].returncode == 0, 'Adapter did not stop cleanly'
    n.destroy_node()
    rclpy.shutdown()
