"""Synthetic, isolated-domain rate test. Never publishes cmd_vel or opens hardware.

Run inside a sourced ROS container: ROS_DOMAIN_ID=231 python3 this_file.py
This checks configuration/runtime throughput, not real sensor accuracy/latency.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

if os.environ.get('ROS_DOMAIN_ID') != '231':
    raise SystemExit('Set ROS_DOMAIN_ID=231 to isolate this synthetic test')
config = Path(__file__).resolve().parents[1] / 'config'
processes = []
logs = []
rclpy.init()
node = rclpy.create_node('ekf_synthetic_probe')
counts = {'local': [], 'global': [], 'gps': []}


def record(key):
    return lambda message: counts[key].append(time.monotonic())


subscriptions = [node.create_subscription(
    Odometry, '/odom/' + key, record(key), 10) for key in counts]
imu_pub = node.create_publisher(Imu, '/imu/raw', 10)
odom_pub = node.create_publisher(Odometry, '/odom/mavros', 10)
gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)

try:
    specifications = [
        ('ekf_node', 'ekf_local', 'ekf_local.yaml',
         ['odometry/filtered:=/odom/local']),
        ('ekf_node', 'ekf_global', 'ekf_global.yaml',
         ['odometry/filtered:=/odom/global']),
        ('navsat_transform_node', 'navsat_transform', 'navsat_transform.yaml',
         ['imu:=/imu/raw', 'imu/data:=/imu/raw', 'gps/fix:=/gps/fix',
          'odometry/filtered:=/odom/global', 'odometry/gps:=/odom/gps']),
    ]
    for executable, name, filename, remaps in specifications:
        binary = Path(get_package_prefix('robot_localization')) / 'lib/robot_localization' / executable
        command = [str(binary),
                   '--ros-args', '--params-file', str(config / filename),
                   '-r', '__node:=' + name]
        for remap in remaps:
            command.extend(['-r', remap])
        log = tempfile.TemporaryFile(mode='w+')
        logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=log))
    start = time.monotonic()
    tick = 0
    def publish_sensors():
        global tick
        stamp = node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.w = 1.0
        imu.orientation_covariance[8] = .01
        imu.angular_velocity_covariance[8] = .01
        imu_pub.publish(imu)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        odom.twist.covariance[0] = .1
        odom.twist.covariance[7] = .1
        odom_pub.publish(odom)
        if tick % 5 == 0:
            gps = NavSatFix()
            gps.header.stamp = stamp
            gps.header.frame_id = 'base_link'
            gps.status.status = 0
            gps.status.service = 1
            gps.latitude, gps.longitude, gps.altitude = 42.0, -71.0, 0.0
            gps.position_covariance = [.25, 0., 0., 0., .25, 0., 0., 0., 1.]
            gps.position_covariance_type = 2
            gps_pub.publish(gps)
        tick += 1
    timer = node.create_timer(.02, publish_sensors)
    while time.monotonic() - start < 15:
        rclpy.spin_once(node, timeout_sec=.01)
    assert all(process.poll() is None for process in processes), 'Filter process exited'
    for key, minimum in (('local', 40), ('global', 15), ('gps', 5)):
        stamps = [stamp for stamp in counts[key] if stamp > start + 8]
        assert len(stamps) > 2, f'No usable {key} output'
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(f'{key}: {hz:.1f} Hz ({len(stamps)} messages after warmup)', flush=True)
        assert hz >= minimum, f'{key} output too slow: {hz}'
finally:
    for process in processes:
        process.terminate()
    for process in processes:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    for log in logs:
        log.seek(0)
        print(log.read()[-3000:])
        log.close()
    node.destroy_node()
    rclpy.shutdown()
