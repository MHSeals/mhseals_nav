"""lidar_fusion node: TF, camera_info, a timed cloud and a boxed observation -> fused range."""
import math
from pathlib import Path

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
def rig():
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

    def tick():
        tf = TransformStamped()
        tf.header.stamp = peer.get_clock().now().to_msg()
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
