"""ZED backend: native ObjectsStamped -> /perception/observations, one array per camera frame.

The pure `convert_objects` is shared with the sim backend's ZED emulation.
"""

import math
from socket import gethostname
from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo

from mhseals_nav import horizon, perception_status, waterline
from mhseals_nav.msg import Observation, ObservationArray
from mhseals_nav.utils.live_params import LiveParams, positive
from mhseals_nav.yaml_params import node_config

VELOCITY_KINDS = {'none': Observation.VELOCITY_NONE,
                  'relative': Observation.VELOCITY_RELATIVE,
                  'world': Observation.VELOCITY_WORLD}
TRACKING_OK = 1  # sl::OBJECT_TRACKING_STATE::OK; SEARCHING and TERMINATE are predictions


def finite(values, count):
    out = [float(v) for v in values]
    if len(out) != count or not all(map(math.isfinite, out)):
        raise ValueError(f'expected {count} finite values')
    return out


def covariance(upper):
    """ZED's 6 upper-triangular values (xx xy xz yy yz zz) -> row-major 3x3."""
    xx, xy, xz, yy, yz, zz = finite(upper, 6)
    return [xx, xy, xz, xy, yy, yz, xz, yz, zz]


def box2d(corners, camera_size):
    """Pixel corners -> normalized u0 v0 u1 v1, or None without a usable image size or box."""
    if not camera_size or len(corners) != 4:
        return None
    width, height = camera_size
    us = [float(c['kp'][0]) for c in corners]
    vs = [float(c['kp'][1]) for c in corners]
    if width <= 0 or height <= 0 or max(us) <= min(us) or max(vs) <= min(vs):
        return None
    clamp = lambda value: max(0.0, min(1.0, value))  # noqa: E731
    return [clamp(min(us) / width), clamp(min(vs) / height),
            clamp(max(us) / width), clamp(max(vs) / height)]


def observation(obj, camera_size, kind):
    out = Observation()
    out.class_id = str(obj['label'])
    out.score = max(0.0, min(1.0, float(obj.get('confidence', 0.0)) / 100.0))
    label_id = int(obj.get('label_id', -1))
    out.backend_id = str(label_id) if obj.get('tracking_available', False) and label_id >= 0 \
        else ''
    centre = finite(obj['position'], 3)
    size = obj.get('dimensions_3d', [0.0, 0.0, 0.0])
    size = [float(v) for v in size] if len(size) == 3 and all(
        math.isfinite(float(v)) and float(v) >= 0 for v in size) else [0.0, 0.0, 0.0]
    corners = obj.get('bounding_box_3d', {}).get('corners', [])
    if len(corners) == 8:
        points = [finite(corner['kp'], 3) for corner in corners]
        low = [min(p[i] for p in points) for i in range(3)]
        high = [max(p[i] for p in points) for i in range(3)]
        # SDK corners are already in the reported ROS frame. Enclose them there, rather
        # than assuming width/height/depth match its XYZ axes.
        if high != low:
            centre = [(a + b) / 2 for a, b in zip(low, high)]
            size = [b - a for a, b in zip(low, high)]
    out.position.x, out.position.y, out.position.z = centre
    out.size.x, out.size.y, out.size.z = size
    if any(obj.get('position_covariance', [])):
        out.position_covariance = covariance(obj['position_covariance'])
    out.velocity_kind = kind
    if kind != Observation.VELOCITY_NONE:
        out.velocity.x, out.velocity.y, out.velocity.z = finite(obj['velocity'], 3)
    box = box2d(obj.get('bounding_box_2d', {}).get('corners', []), camera_size)
    out.has_box2d = box is not None
    if box is not None:
        out.box2d = box
    return out


def convert_objects(packet, camera_size, velocity_kind):
    """message_to_ordereddict(ObjectsStamped) -> ObservationArray.

    `camera_size` is the (width, height) the 2D box pixels refer to, or None.
    `velocity_kind` is 'none', 'relative' or 'world'. The caller sets `backend` and
    `processing_latency`. Objects the SDK only predicts (tracking state not OK) are skipped,
    and so is any object with non-finite numbers; the frame itself is never dropped here.
    """
    kind = VELOCITY_KINDS[velocity_kind]
    array = ObservationArray()
    header = packet['header']
    frame = header['frame_id']
    if not isinstance(frame, str) or not frame or len(frame) > 256:
        raise ValueError('ZED header must name its actual coordinate frame')
    array.header.frame_id = frame
    array.header.stamp.sec = int(header['stamp']['sec'])
    array.header.stamp.nanosec = int(header['stamp']['nanosec'])
    if not 0 <= array.header.stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid ZED timestamp')
    objects = packet['objects']
    if not isinstance(objects, list) or len(objects) > 256:
        raise ValueError('expected at most 256 ZED objects')
    for obj in objects:
        if obj.get('tracking_available', False) and obj.get('tracking_state') != TRACKING_OK:
            continue
        try:
            array.observations.append(observation(obj, camera_size, kind))
        except (ValueError, TypeError, OverflowError):
            continue
    return array


class ZedBackend(Node):
    def __init__(self):
        super().__init__('zed_backend', automatically_declare_parameters_from_overrides=True)
        # Every default lives in config/zed_backend.yaml; launch overrides sit on top.
        cfg = node_config(self, 'zed_backend.yaml')
        if not self.has_parameter('max_detection_age'):  # LiveParams needs it declared
            self.declare_parameter('max_detection_age', cfg['max_detection_age'])
        self.backend = cfg['backend']
        self.velocity_kind = cfg['velocity_kind']
        if self.velocity_kind not in VELOCITY_KINDS:
            raise ValueError(f'velocity_kind must be one of {sorted(VELOCITY_KINDS)}')
        self.box2d_scale = cfg['box2d_scale']
        self.fov = (cfg['fov']['hfov_deg'], cfg['fov']['range_m'])
        self.live = LiveParams(
            self, {'max_detection_age': positive},
            lambda values, changed: setattr(self, 'max_age', values['max_detection_age']))
        self.max_age = self.live['max_detection_age']
        self.camera_size = None
        self.K = self.info_size = None
        self.frame = ''
        self.ids = False
        self.last_stamp = -1
        self.stage = perception_status.StageClock()
        # Per-frame camera attitude (ZED IMU, horizon) and the waterline range it enables.
        self.att_cfg, self.hz_cfg, self.wl_cfg = cfg['attitude'], cfg['horizon'], cfg['waterline']
        self.attitude = waterline.Attitude(self.att_cfg)
        self.mount = waterline.mount_matrix(self.att_cfg['imu_mount_rpy_deg'])
        self.counts = {'images': 0, 'fixes': 0, 'mono': 0, 'boxes': 0}
        self.att_source = 'none'
        self.pub = self.create_publisher(
            ObservationArray, '/perception/observations',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_pub = self.create_publisher(DiagnosticArray, '/perception/status', 1)
        self.report_timer = self.create_timer(1.0, self.report)
        try:
            from zed_msgs.msg import ObjectsStamped
        except ImportError:
            from zed_interfaces.msg import ObjectsStamped
        self.create_subscription(CameraInfo, cfg['camera_info_topic'],
                                 self.camera_info, qos_profile_sensor_data)
        self.create_subscription(ObjectsStamped, cfg['objects_topic'],
                                 self.objects, qos_profile_sensor_data)
        if self.att_cfg['enabled']:
            from sensor_msgs.msg import Imu
            self.create_subscription(Imu, self.att_cfg['imu_topic'], self.imu,
                                     qos_profile_sensor_data)
        if self.att_cfg['enabled'] and self.hz_cfg['enabled']:
            from sensor_msgs.msg import Image
            self.create_subscription(Image, self.hz_cfg['image_topic'], self.image,
                                     QoSProfile(depth=1,
                                                reliability=ReliabilityPolicy.BEST_EFFORT))

    def camera_info(self, msg):
        # ZED 2D boxes are in grab pixels; camera_info is at the (downscaled) publish size.
        self.camera_size = (msg.width * self.box2d_scale, msg.height * self.box2d_scale)
        k = getattr(msg, 'k', None)
        if k is not None and k[0] > 0 and k[4] > 0:   # an uncalibrated info leaves K unset
            self.K = (k[0], k[4], k[2], k[5])
            self.info_size = (msg.width, msg.height)

    def imu(self, msg):
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        roll, pitch = waterline.camera_roll_pitch((q.x, q.y, q.z, q.w), self.mount)
        self.attitude.add_imu(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                              roll, pitch)

    def image(self, msg):
        """Fold a horizon fix from the published colour frame into the attitude."""
        self.counts['images'] += 1
        if self.K is None or self.counts['images'] % max(int(self.hz_cfg['every_n']), 1) or \
                (msg.width, msg.height) != self.info_size:
            return
        channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.step // channels, channels)[:, :msg.width]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prior = self.attitude.at(t)
        fix = horizon.detect(frame, self.K, self.hz_cfg,
                             prior=None if prior is None else prior[:2])
        if fix is None or abs(fix['roll']) > math.radians(self.hz_cfg['max_roll_deg']):
            return
        self.counts['fixes'] += 1
        self.attitude.add_horizon(t, fix['roll'], fix['pitch'], fix['sigma'])

    def objects(self, msg):
        packet = message_to_ordereddict(msg)
        if packet['objects']:  # an empty frame says nothing about ids
            self.ids = any(o.get('tracking_available', False) for o in packet['objects'])
        self.publish_packet(packet)

    def publish_packet(self, packet):
        try:
            array = convert_objects(packet, self.camera_size, self.velocity_kind)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.stage.drop('malformed')
            self.get_logger().warning(f'Dropped malformed ZED objects: {error}',
                                      throttle_duration_sec=5.0)
            return
        stamp = array.header.stamp.sec * 1_000_000_000 + array.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        age = (now - stamp) * 1e-9
        if stamp <= self.last_stamp:
            self.stage.drop('reorder')
            return
        if not -.1 <= age <= self.max_age:
            self.stage.drop('stale')
            return
        self.last_stamp = stamp
        self.frame = array.header.frame_id
        if self.att_cfg['enabled'] and self.wl_cfg['enabled']:
            estimate = self.attitude.at(stamp * 1e-9)
            self.att_source = estimate[3] if estimate else 'none'
            self.counts['boxes'] += sum(ob.has_box2d for ob in array.observations)
            self.counts['mono'] += waterline.fill_mono(
                array.observations, self.K, self.info_size, estimate, self.wl_cfg)
        array.backend = self.backend
        array.processing_latency = max(age, 0.0)
        self.pub.publish(array)
        self.stage.accept(monotonic(), age, len(array.observations))

    def report(self):
        """1 Hz /perception/status stage perception/<backend>/backend."""
        values = self.stage.values(monotonic())
        values.update(hfov_deg=f'{self.fov[0]:g}', range_m=f'{self.fov[1]:g}',
                      frame=self.frame, ids=str(self.ids).lower(),
                      velocity=self.velocity_kind, attitude=self.att_source,
                      horizon_fixes=self.counts['fixes'], horizon_images=self.counts['images'],
                      mono_share=f"{self.counts['mono'] / max(self.counts['boxes'], 1):.2f}")
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [perception_status.status('backend', gethostname(), values,
                                                 source=self.backend)]
        self.status_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ZedBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
