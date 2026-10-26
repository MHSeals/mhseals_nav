"""lidar_fusion node: TF, camera_info, a timed cloud and a boxed observation -> fused range."""
import math
from pathlib import Path
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402, I100
from geometry_msgs.msg import TransformStamped  # noqa: E402
from mhseals_nav.lidar_fusion import BODY_TO_OPTICAL, LidarFusion  # noqa: E402
from mhseals_nav.msg import Observation, ObservationArray  # noqa: E402
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2, PointField  # noqa: E402
from test_lidar_fusion import box_of, scan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'lidar_fusion.yaml'
W, H, F = 1280, 720, 640.0


def cloud(points, stamp):
    """Body-axes PointCloud2 with x y z time (time spread over -0.1..0 s)."""
    data = np.zeros(len(points), [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('time', '<f4')])
    data['x'], data['y'], data['z'] = points.T
    data['time'] = np.linspace(-0.1, 0.0, len(points))
    msg = PointCloud2(height=1, width=len(points), point_step=16, row_step=16 * len(points),
                      is_dense=True, data=data.tobytes())
    msg.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'time'))]
    msg.header.stamp, msg.header.frame_id = stamp, 'lidar_link'
    return msg


@pytest.fixture
def rig(request):
    rclpy.init(args=['--ros-args', '--params-file', str(CONFIG), '-p', 'points_transport:=direct',
                     '-p', 'camera_info_topic:=/cam/info', '-p', 'camera_axes:=body'])
    node, peer = LidarFusion(), rclpy.create_node('lidar_fusion_peer')
    got = []
    lossy = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    peer.create_subscription(ObservationArray, '/perception/observations_fused', got.append,
                             lossy)
    peer.create_subscription(DiagnosticArray, '/perception/status', lambda m: None, 10)
    statics = []
    for child in ('lidar_link', 'front_camera_link'):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', child
        tf.transform.translation.x, tf.transform.translation.z = 0.5, 1.0
        tf.transform.rotation.w = 1.0
        statics.append(tf)
    StaticTransformBroadcaster(peer).sendTransform(statics)
    moving = TransformBroadcaster(peer)
    imu = peer.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)

    age = int(getattr(request, 'param', {}).get('tf_age_s', 0.0) * 1e9)    # stale odom TF

    def tick():
        tf = TransformStamped()
        tf.header.stamp = rclpy.time.Time(
            nanoseconds=peer.get_clock().now().nanoseconds - age).to_msg()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.transform.rotation.w = 1.0
        moving.sendTransform(tf)
        imu.publish(Imu())
    peer.create_timer(0.01, tick)
    info = peer.create_publisher(CameraInfo, '/cam/info', qos_profile_sensor_data)
    points = peer.create_publisher(PointCloud2, '/points', qos_profile_sensor_data)
    obs = peer.create_publisher(ObservationArray, '/perception/observations', lossy)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    yield node, peer, executor, info, points, obs, got
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


def test_a_biased_stereo_buoy_comes_out_on_the_lidar_range(rig):
    node, peer, executor, info, points, obs, got = rig
    X, Z = 1.0, 8.0                                   # optical axes, from the camera
    target = (X, Z, 0.2, 0.4, 1.0)                    # y down: top 0.4 m below, water 1.0 m
    body = scan([target]) @ BODY_TO_OPTICAL           # optical -> body (lidar_link)
    k = CameraInfo(width=W, height=H)
    k.k = [F, 0.0, W / 2, 0.0, F, H / 2, 0.0, 0.0, 1.0]
    end = peer.get_clock().now().nanoseconds + 4e9
    sent = 0
    while peer.get_clock().now().nanoseconds < end and not node.counts['lidar']:
        now = peer.get_clock().now()
        info.publish(k)
        points.publish(cloud(body, now.to_msg()))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)
        o = Observation(class_id='red_buoy', score=0.9, backend_id='3', has_box2d=True)
        c = BODY_TO_OPTICAL.T @ (np.array([X, 0.7, Z]) * 1.12)
        o.position.x, o.position.y, o.position.z = map(float, c)
        o.box2d = [float(b) for b in box_of(*target)]
        a = ObservationArray(backend='sim', observations=[o])
        a.header.stamp = (now - rclpy.duration.Duration(seconds=0.03)).to_msg()
        a.header.frame_id = 'front_camera_link'
        obs.publish(a)
        sent += 1
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
    assert got, f'no fused output after {sent} frames'
    fused = [a for a in got if a.observations]
    last = fused[-1].observations[0]
    assert node.counts['lidar'] >= 1, (node.counts, node.skips)
    assert math.hypot(last.position.x, last.position.y) == pytest.approx(
        math.hypot(X, Z), abs=0.1)
    assert last.backend_id == '3' and fused[-1].header.frame_id == 'front_camera_link'


@pytest.fixture
def bare():
    """Provide the node alone: no TF, no camera_info, no cloud unless a test sets them."""
    rclpy.init(args=['--ros-args', '--params-file', str(CONFIG), '-p', 'points_transport:=direct',
                     '-p', 'camera_axes:=body'])
    node = LidarFusion()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def frame(node, stamp_offset_s=0.0, cov=(0.25, 0.01, 0.0025), body=(8.0, 1.0, -0.7)):
    """One boxed body-axes observation from front_camera_link, stamped now + offset."""
    o = Observation(class_id='red_buoy', score=0.9, backend_id='3', has_box2d=True)
    o.position.x, o.position.y, o.position.z = body
    o.position_covariance = np.diag(cov).reshape(-1).tolist()
    o.box2d = [0.45, 0.45, 0.5, 0.55]
    a = ObservationArray(backend='sim', observations=[o])
    a.header.stamp = rclpy.time.Time(
        nanoseconds=node.get_clock().now().nanoseconds + int(stamp_offset_s * 1e9)).to_msg()
    a.header.frame_id = 'front_camera_link'
    return a


def known_camera(node):
    node.K, node.image = (F, F, W / 2, H / 2), (W, H)
    node._camera, node.A = (BODY_TO_OPTICAL.T, np.zeros(3)), BODY_TO_OPTICAL


def tangential_var(ob):
    """Body-axes horizontal variance across the ray (z up)."""
    u = np.array([ob.position.x, ob.position.y, 0.0])
    u /= np.linalg.norm(u)
    n = np.array([-u[1], u[0], 0.0])
    C = np.array(ob.position_covariance).reshape(3, 3)
    return float(n @ C @ n)


def test_no_camera_info_nor_camera_tf_passes_stereo_box_less_in_body_axes(bare):
    msg = frame(bare)
    out = bare.process(msg)
    assert bare.last_skip == 'no_camera' and bare.sources == ['stereo']
    ob, src = out.observations[0], msg.observations[0]
    assert (ob.position.x, ob.position.y, ob.position.z) == pytest.approx(
        (src.position.x, src.position.y, src.position.z))
    # Box-less keeps the stereo tangential sigma (K = (1, 1, 0, 0) made it ~rho), and the
    # body axes come from the frame name, not eye(3) (z up stays up).
    assert tangential_var(ob) == pytest.approx(tangential_var(src), rel=1e-6)
    C = np.array(ob.position_covariance).reshape(3, 3)
    assert C[2, 2] == pytest.approx(0.0025, rel=1e-6)


def test_no_lidar_frames_leave_on_stereo_once_the_wait_ends(bare):
    known_camera(bare)
    bare.on_frame(frame(bare))
    assert len(bare.queue) == 1                 # waits for a scan, as for TF
    time.sleep(bare.cfg['tf_wait_s'] + 0.05)
    bare.drain()
    assert not bare.queue and bare.skips.get('no_scan') == 1
    assert bare.sources == ['stereo'] and bare.counts['lidar'] == 0


def test_a_future_stamp_does_not_hold_the_queue_past_the_wait(bare):
    known_camera(bare)
    bare.on_frame(frame(bare, stamp_offset_s=30.0))     # clock mismatch: ROS age stays < 0
    assert len(bare.queue) == 1
    time.sleep(bare.cfg['tf_wait_s'] + 0.05)
    bare.drain()
    assert not bare.queue and bare.skips.get('no_scan') == 1


@pytest.mark.parametrize('rig', [{'tf_age_s': 2.0}], indirect=True)
def test_stale_tf_skips_after_the_wait_and_keeps_stereo(rig):
    node, peer, executor, info, points, obs, got = rig
    target = (1.0, 8.0, 0.2, 0.4, 1.0)
    body = scan([target]) @ BODY_TO_OPTICAL
    k = CameraInfo(width=W, height=H)
    k.k = [F, 0.0, W / 2, 0.0, F, H / 2, 0.0, 0.0, 1.0]
    end = peer.get_clock().now().nanoseconds + 3e9
    while peer.get_clock().now().nanoseconds < end and not node.skips.get('tf_missing'):
        now = peer.get_clock().now()
        info.publish(k)
        points.publish(cloud(body, now.to_msg()))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)
        a = frame(peer, -0.03)
        a.observations[0].box2d = [float(b) for b in box_of(*target)]
        obs.publish(a)
        for _ in range(30):
            executor.spin_once(timeout_sec=0.01)
    assert node.skips.get('tf_missing'), node.skips
    assert node.counts['lidar'] == 0 and node.counts['stereo'] >= 1
    assert len(node.queue) <= 1
