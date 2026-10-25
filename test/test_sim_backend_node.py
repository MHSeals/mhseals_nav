"""sim_backend node: its YAML loads, live injection reaches the published observations."""
from importlib.util import find_spec
import inspect
import json
import math
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402, I100
from geometry_msgs.msg import TransformStamped  # noqa: E402
from mhseals_nav.msg import ObservationArray  # noqa: E402
from mhseals_nav.sim_backend import SimBackend  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose  # noqa

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'sim_backend.yaml'


def _nav_camera():
    """zed_backend and StageClock.accept(objects) land with nav-camera."""
    from mhseals_nav.perception_status import StageClock
    return find_spec('mhseals_nav.zed_backend') is not None and \
        'objects' in inspect.signature(StageClock.accept).parameters


needs_nav_camera = pytest.mark.skipif(not _nav_camera(),
                                      reason='needs zed_backend and the trimmed StageClock')


@pytest.fixture
def rig(tmp_path):
    truth = tmp_path / 'truth.json'
    truth.write_text(json.dumps({'elements': [{'label': 'red_buoy', 'x': 5.0, 'y': 0.0}]}))
    rclpy.init(args=['--ros-args', '--params-file', str(CONFIG),
                     '-p', f'course_truth:={truth}'])
    backend, peer = SimBackend(), rclpy.create_node('sim_backend_peer')
    got = {'obs': [], 'status': []}
    peer.create_subscription(ObservationArray, '/perception/observations', got['obs'].append,
                             QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    peer.create_subscription(DiagnosticArray, '/perception/status', got['status'].append, 10)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'front_camera_link'
    tf.transform.translation.x = 1.0
    tf.transform.rotation.w = 1.0
    StaticTransformBroadcaster(peer).sendTransform(tf)
    odom = peer.create_publisher(Odometry, '/crane/odom', 10)
    dets = peer.create_publisher(Detection3DArray, '/detections', qos_profile_sensor_data)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(backend)
    executor.add_node(peer)

    def frame():
        """Send truth (boat at the origin facing +x) and one buoy 4 m ahead; wait for output."""
        stamp = peer.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.pose.pose.orientation.w = stamp, 1.0
        odom.publish(o)
        d = Detection3D()
        d.results.append(ObjectHypothesisWithPose())
        d.results[0].hypothesis.class_id, d.results[0].hypothesis.score = 'red_buoy', 0.9
        d.bbox.center.position.x, d.bbox.size.x = 4.0, 0.4
        msg = Detection3DArray(detections=[d])
        msg.header.stamp, msg.header.frame_id = stamp, 'front_camera_link'
        count = len(got['obs'])
        for _ in range(100):
            if len(got['obs']) == count:
                dets.publish(msg)
            executor.spin_once(timeout_sec=0.05)
            if len(got['obs']) > count:
                return got['obs'][-1]
        raise AssertionError('no observations')

    yield backend, frame, got
    executor.shutdown()
    backend.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


def buoys(array):
    """Keep the course buoy; the YAML's phantom boat may be in view too."""
    return [o for o in array.observations if o.class_id == 'red_buoy']


@needs_nav_camera
def test_plain_then_zed_with_a_heading_error_set_live(rig):
    backend, frame, got = rig
    out = frame()
    assert out.backend == 'sim' and out.header.frame_id == 'front_camera_link'
    [o] = buoys(out)
    assert (o.position.x, o.position.y, o.backend_id) == pytest.approx((4.0, 0.0, ''))
    backend.set_parameters([Parameter('emulate', value='zed'),
                            Parameter('inject.heading_bias_deg', value=90.0)])
    [o] = buoys(frame())
    # The buoy sits 5 m ahead of base_link; a 90 deg heading error swings it to the left.
    assert (o.position.x, o.position.y) == pytest.approx((-1.0, 5.0), abs=1e-5)
    assert o.backend_id == '0'
    frame()  # the inject echo goes out before the observations, on another topic
    echoed = [{v.key: v.value for v in s.values} for a in got['status'] for s in a.status
              if s.name == 'perception/sim/inject']
    assert any(e['heading_bias_deg'] == '90' and e['emulate'] == 'zed' for e in echoed)


def test_bad_live_values_are_refused(rig):
    backend, _, _ = rig
    results = backend.set_parameters([Parameter('emulate', value='lidar'),
                                      Parameter('case', value='nope')])
    assert not any(r.successful for r in results)
    assert math.isclose(backend.cfg['inject']['heading_bias_deg'], 0.0)
