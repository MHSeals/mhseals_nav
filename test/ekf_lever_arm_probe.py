"""Pure yaw spin in place: the fused base_link must not move. A lever-arm error shows up here.

Synthetic and isolated-domain, like ekf_runtime_probe.py; never publishes cmd_vel or opens
hardware. Run inside a sourced ROS container with this package built:
    ROS_DOMAIN_ID=231 python3 test/ekf_lever_arm_probe.py [fix_frame [twist_frame]]
The boat spins at 1 rad/s about base_link at a fixed spot. The GPS antenna (gps_link in the
URDF) circles it, so the fix and the receiver velocity are the antenna's, labelled as the
odometry adapter labels them: gps_link for both (before odom-100hz the fix said base_link).
It prints the spread of /odom/global and /odom/local positions over the spin and, with both
in gps_link, fails above 5 cm.
On 2026-10-25: both gps_link 1.0 cm global, 0.0 local; fix base_link 15 cm global.
"""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
import yaml

if os.environ.get('ROS_DOMAIN_ID') != '231':
    raise SystemExit('Set ROS_DOMAIN_ID=231 to isolate this synthetic test')
GPS_FRAME = sys.argv[1] if len(sys.argv) > 1 else 'gps_link'
TWIST_FRAME = sys.argv[2] if len(sys.argv) > 2 else GPS_FRAME   # the adapter's child frame
ROOT = Path(__file__).resolve().parents[1]
config = ROOT / 'config'
W, STILL, SPIN = 1.0, 8.0, 20.0           # rad/s, s still (navsat datum), s spinning
LAT0, LON0, EARTH = 27.3747, -82.4518, 6378137.0

urdf = subprocess.run(['xacro', str(ROOT / 'description/blastoise/blastoise.xacro')],
                      check=True, capture_output=True, text=True).stdout
antenna = None
for line in urdf.split('<joint')[1:]:
    if 'name="gps_link_joint"' in line:
        xyz = line.split('xyz="')[1].split('"')[0].split()
        antenna = (float(xyz[0]), float(xyz[1]))
assert antenna, 'gps_link_joint not in the URDF'

processes, logs = [], []
rclpy.init()
node = rclpy.create_node('ekf_lever_arm_probe')
poses = {'global': [], 'local': []}
for key in poses:
    node.create_subscription(Odometry, '/odom/' + key, lambda m, k=key: poses[k].append(
        (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y)), 50)
imu_pub = node.create_publisher(Imu, '/imu/raw', 10)
odom_pub = node.create_publisher(Odometry, '/odom/mavros', 10)
gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)


def start(package, executable, name, params, remaps=()):
    binary = Path(get_package_prefix(package)) / 'lib' / package / executable
    command = [str(binary), '--ros-args', '--params-file', str(params), '-r', '__node:=' + name]
    for remap in remaps:
        command += ['-r', remap]
    log = tempfile.TemporaryFile(mode='w+')
    logs.append(log)
    processes.append(subprocess.Popen(command, stdout=log, stderr=log))


try:
    description = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf}}},
                   description)
    description.close()
    start('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
          description.name)
    start('robot_localization', 'ekf_node', 'ekf_local', config / 'ekf_local.yaml',
          ['odometry/filtered:=/odom/local'])
    start('robot_localization', 'ekf_node', 'ekf_global', config / 'ekf_global.yaml',
          ['odometry/filtered:=/odom/global'])
    start('robot_localization', 'navsat_transform_node', 'navsat_transform',
          config / 'navsat_transform.yaml',
          ['imu:=/imu/raw', 'imu/data:=/imu/raw', 'gps/fix:=/gps/fix',
           'odometry/filtered:=/odom/global', 'odometry/gps:=/odom/gps'])
    t0 = time.monotonic()
    tick = [0]

    def yaw(t):
        return W * max(0., t - STILL)

    def publish():
        t = time.monotonic() - t0
        psi, rate = yaw(t), (W if t > STILL else 0.)
        stamp = node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        imu.orientation.z, imu.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
        imu.angular_velocity.z = rate
        imu.orientation_covariance = [.04, 0., 0., 0., .04, 0., 0., 0., .04]
        imu.angular_velocity_covariance = [1.2e-7, 0., 0., 0., 1.2e-7, 0., 0., 0., 1.2e-7]
        imu_pub.publish(imu)
        tick[0] += 1
        if tick[0] % 20:
            return                                    # GPS and its velocity at 5 Hz
        ax, ay = antenna
        east = math.cos(psi) * ax - math.sin(psi) * ay
        north = math.sin(psi) * ax + math.cos(psi) * ay
        fix = NavSatFix()
        fix.header.stamp, fix.header.frame_id = stamp, GPS_FRAME
        fix.status.status, fix.status.service = 0, 1
        fix.latitude = LAT0 + math.degrees(north / EARTH)
        fix.longitude = LON0 + math.degrees(east / (EARTH * math.cos(math.radians(LAT0))))
        fix.position_covariance = [.0004, 0., 0., 0., .0004, 0., 0., 0., .01]
        fix.position_covariance_type = 2
        gps_pub.publish(fix)
        odom = Odometry()                             # the antenna's velocity, body axes
        odom.header.stamp, odom.header.frame_id = stamp, 'map'
        odom.child_frame_id = TWIST_FRAME
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = imu.orientation.z, imu.orientation.w
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = -rate * ay, rate * ax
        for i in (0, 7, 14):
            odom.twist.covariance[i] = .05 ** 2
        odom_pub.publish(odom)

    node.create_timer(.01, publish)
    while time.monotonic() - t0 < STILL + SPIN:
        rclpy.spin_once(node, timeout_sec=.005)
    assert all(p.poll() is None for p in processes), 'a node exited'
    spreads = {}
    for key, samples in poses.items():
        spin = [(x, y) for t, x, y in samples if t - t0 > STILL + 3]
        assert len(spin) > 10, f'no /odom/{key} during the spin'
        mx, my = sum(x for x, _ in spin) / len(spin), sum(y for _, y in spin) / len(spin)
        spreads[key] = max(math.hypot(x - mx, y - my) for x, y in spin)
        print(f'fix {GPS_FRAME}, twist {TWIST_FRAME}: /odom/{key} position spread over a {W:g} rad/s spin '
              f'{spreads[key] * 100:.1f} cm (antenna lever arm {math.hypot(*antenna) * 100:.1f} cm)',
              flush=True)
    if GPS_FRAME == TWIST_FRAME == 'gps_link':
        assert spreads['global'] < .05, spreads
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
        text = log.read()
        if 'rror' in text or 'arn' in text:
            print(text[-1500:])
        log.close()
    node.destroy_node()
    rclpy.shutdown()
