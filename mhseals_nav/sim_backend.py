"""Sim camera backend: Unity /detections + crane truth -> /perception/observations.

Injects heading, lag, extrinsic and stamp errors (live params) before conversion, then emits
`plain` observations or, in `zed` mode, a ZED-shaped packet through `zed_backend.convert_objects`
so the boat's own conversion code runs. Everything above the node class is pure (numpy only).

Sign convention: the boat's estimated heading is truth + heading_bias - heading_lag * yaw_rate
(a heading that lags truth by heading_lag). Each observation is rotated about base_link by that
heading error, which is what lifting with the wrong heading does, so the tracker's heading check
estimates lag_s = +heading_lag.
"""

import json
import math
from socket import gethostname
from time import monotonic

import numpy as np

try:
    from rclpy.node import Node
except ImportError:  # host pytest exercises only the pure functions
    Node = object

INJECT_KEYS = ('heading_bias_deg', 'heading_lag_s', 'extrinsic_yaw_deg', 'stamp_offset_s',
               'extrinsic_roll_deg', 'extrinsic_pitch_deg')
# Detector emulation (inject.*, read per frame; not part of cases): see noisy().
NOISE_KEYS = ('depth_k', 'box_px', 'merge_gap_px')
PHANTOM_ID = 1000  # ZED label_id of phantom i is PHANTOM_ID + i; truth ids are element indices


def rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def nest(flat):
    """{'a.b': 1} -> {'a': {'b': 1}} (ROS parameter names to the YAML shape)."""
    out = {}
    for name, value in flat.items():
        *parents, leaf = name.split('.')
        node = out
        for key in parents:
            node = node.setdefault(key, {})
        node[leaf] = value
    return out


def injection(cfg, case, elapsed):
    """Active inject values: cfg['inject'] when case is none, else the case row at `elapsed` s."""
    if case == 'none':
        return {k: float(cfg['inject'][k]) for k in INJECT_KEYS}
    rows = cfg['cases'][case]
    index = max([0] + [i for i, at in enumerate(rows['at_s']) if at <= elapsed])
    return {k: float(rows[k][index]) if k in rows else 0.0 for k in INJECT_KEYS}


def rx(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def heading_error(inj, yaw_rate):
    """Estimated minus true heading [rad]."""
    return math.radians(inj['heading_bias_deg']) - inj['heading_lag_s'] * yaw_rate


def inject(objects, R_bc, t_bc, heading, extrinsic, tilt=(0.0, 0.0), offset=(0.0, 0.0, 0.0)):
    """Objects (camera frame) as the boat sees them with the given heading and extrinsic errors.

    Heading: p' = T_bc^-1 Rz(heading) T_bc p, a turn about base_link. Extrinsic: a turn about
    the camera's own origin, Rz(extrinsic) Ry(pitch) Rx(roll) in base axes with tilt = (roll,
    pitch) [rad], then ``offset`` [m, base axes] added to every point. Velocities turn with the
    same rotations.
    """
    camera = R_bc.T @ rz(extrinsic) @ ry(tilt[1]) @ rx(tilt[0]) @ R_bc
    shift = R_bc.T @ np.asarray(offset, float)
    turn = R_bc.T @ rz(heading) @ R_bc @ camera
    out = []
    for obj in objects:
        p = R_bc.T @ (rz(heading) @ (R_bc @ (camera @ obj['p'] + shift) + t_bc) - t_bc)
        out.append(dict(obj, p=p, v=None if obj.get('v') is None else turn @ obj['v']))
    return out


def in_fov(p, R_bc, fov):
    """Tell whether a camera-frame point is in the FOV and range (camera looks along base +x)."""
    q = R_bc @ p
    return (q[0] > 0 and np.linalg.norm(p) <= fov['range_m']
            and abs(math.atan2(q[1], q[0])) <= math.radians(fov['hfov_deg']) / 2)


def to_world(p, pose_wb, R_bc, t_bc):
    R_wb, t_wb = pose_wb
    return R_wb @ (R_bc @ p + t_bc) + t_wb


def to_camera(p_w, pose_wb, R_bc, t_bc):
    R_wb, t_wb = pose_wb
    return R_bc.T @ (R_wb.T @ (p_w - t_wb) - t_bc)


def match_truth(label, xy, elements, max_m):
    """Index of the nearest same-label truth element within max_m of xy, else None."""
    best, index = max_m, None
    for i, e in enumerate(elements):
        d = math.hypot(e['x'] - xy[0], e['y'] - xy[1])
        if e['label'] == label and d <= best:
            best, index = d, i
    return index


def phantoms(cfg, t):
    """[(label, position, velocity, size)] in the truth world frame at t [s], if enabled."""
    out = []
    if not cfg.get('enabled', True):
        return out
    for name in cfg['names']:
        ph = cfg[name]
        w = ph['speed_mps'] / ph['radius_m']
        a = w * t
        c = np.array([*ph['centre'], 0.0])
        out.append((ph['label'],
                    c + ph['radius_m'] * np.array([math.cos(a), math.sin(a), 0.0]),
                    ph['speed_mps'] * np.array([-math.sin(a), math.cos(a), 0.0]),
                    np.array(ph['size'], float)))
    return out


def scene(detections, t, pose_wb, R_bc, t_bc, cfg, elements):
    """Unity detections plus in-view phantoms, with truth ids and truth velocity.

    `detections` is [(label, score, p_camera, size)]. Without a truth pose (pose_wb None) there
    are no ids and no phantoms. Velocities are ground-fixed in camera axes: 0 for the course.
    """
    objects = []
    for label, score, p, size in detections:
        index = None
        if pose_wb is not None and elements:
            xy = to_world(p, pose_wb, R_bc, t_bc)[:2]
            index = match_truth(label, xy, elements, cfg['truth_match_m'])
        objects.append({'label': label, 'score': score, 'p': np.asarray(p, float),
                        'size': size, 'v': np.zeros(3), 'id': index})
    if pose_wb is None:
        return objects
    for i, (label, p_w, v_w, size) in enumerate(phantoms(cfg['phantoms'], t)):
        p = to_camera(p_w, pose_wb, R_bc, t_bc)
        if in_fov(p, R_bc, cfg['fov']):
            objects.append({'label': label, 'score': 0.9, 'p': p, 'size': size,
                            'v': R_bc.T @ pose_wb[0].T @ v_w, 'id': PHANTOM_ID + i})
    return objects


def project(p, R_bc, K):
    """Camera-frame point -> pixel (u, v) for a camera looking along base +x."""
    q = R_bc @ p
    fx, fy, cx, cy = K
    return cx - fx * q[1] / q[0], cy - fy * q[2] / q[0]


CORNERS = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
           (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]


def corners_of(obj):
    half = np.asarray(obj['size'], float) / 2
    return [obj['p'] + half * np.array(s) for s in CORNERS]


def pixel_box(obj, R_bc, K):
    """Unclipped (u0, v0, u1, v1) around the object's 3D box."""
    uv = np.array([project(c, R_bc, K) for c in corners_of(obj)])
    return (*uv.min(axis=0), *uv.max(axis=0))


def noisy(objects, R_bc, K, cfg, state, rng, t):
    """Emulate the detector after injection: boxes, merges, edge jitter, depth error.

    Boxes come from the injected points; the AR(1) depth error along the ray comes after the
    box is made, so the box keeps the truth. cfg: merge_gap_px (> 0: same-label boxes whose
    column gap is under it merge into the union box at the mean range and direction, keeping
    the nearer one's id), box_px (edge sigma),
    depth_k (range sigma depth_k * rho^2, AR(1) per truth id with time constant depth_tau_s).
    ``state`` maps truth id -> (t, unit AR(1) value) across frames.
    """
    out = [dict(o, box=pixel_box(o, R_bc, K)) for o in objects]
    gap = cfg['merge_gap_px']
    if gap > 0:
        out.sort(key=lambda o: o['box'][0])
        merged = []
        for o in out:
            m = merged[-1] if merged else None
            if m and m['label'] == o['label'] and o['box'][0] - m['box'][2] < gap:
                near, far = (m, o) if np.linalg.norm(m['p']) <= np.linalg.norm(o['p']) else (o, m)
                rho = (np.linalg.norm(m['p']) + np.linalg.norm(o['p'])) / 2
                d = m['p'] / np.linalg.norm(m['p']) + o['p'] / np.linalg.norm(o['p'])
                b = np.array([m['box'], o['box']])
                merged[-1] = dict(near, p=rho * d / np.linalg.norm(d), merged=True,
                                  box=(*b[:, :2].min(axis=0), *b[:, 2:].max(axis=0)),
                                  ids=m.get('ids', [m['id']]) + [o['id']], far=far['id'])
            else:
                merged.append(o)
        out = merged
    for o in out:
        if cfg['box_px'] > 0:
            o['box'] = tuple(np.add(o['box'], rng.normal(0.0, cfg['box_px'], 4)))
        if cfg['depth_k'] > 0:
            key = o['id'] if o['id'] is not None else ('anon', id(o))
            last = state.get(key)
            e = rng.normal()
            if last is not None and o['id'] is not None:
                phi = math.exp(-max(t - last[0], 0.0) / cfg['depth_tau_s'])
                e = phi * last[1] + math.sqrt(1 - phi * phi) * e
            if o['id'] is not None:
                state[key] = (t, e)
            rho = float(np.linalg.norm(o['p']))
            o['p'] = o['p'] * (rho + cfg['depth_k'] * rho ** 2 * e) / rho
    return out


def zed_packet(stamp_ns, frame, objects, R_bc, K, image):
    """message_to_ordereddict(ObjectsStamped)-shaped dict: truth ids, world velocity, 2D box.

    An object's own 'box' (from noisy()) wins over the projection of its 3D box.
    """
    width, height = image
    out = []
    for obj in objects:
        corners = corners_of(obj)
        box = obj.get('box') or pixel_box(obj, R_bc, K)
        u0, v0 = np.clip(box[:2], 0, [width, height]).astype(int)
        u1, v1 = np.clip(box[2:], 0, [width, height]).astype(int)
        box = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        out.append({
            'label': obj['label'], 'label_id': -1 if obj['id'] is None else obj['id'],
            'sublabel': '', 'confidence': 100.0 * obj['score'], 'position': list(obj['p']),
            'position_covariance': [0.0] * 6, 'velocity': list(obj['v']),
            'tracking_available': True, 'tracking_state': 1, 'action_state': 0,
            'bounding_box_2d': {'corners': [{'kp': [u, v]} for u, v in box]},
            'bounding_box_3d': {'corners': [{'kp': list(c)} for c in corners]},
            'dimensions_3d': list(obj['size'])})
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return {'header': {'stamp': {'sec': sec, 'nanosec': nanosec}, 'frame_id': frame},
            'objects': out}


def plain_array(stamp_ns, frame, objects):
    """Build an ObservationArray with no ids, no velocity and no 2D boxes."""
    from mhseals_nav.msg import Observation, ObservationArray
    array = ObservationArray()
    array.header.frame_id = frame
    array.header.stamp.sec, array.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    for obj in objects:
        o = Observation(class_id=obj['label'], score=float(obj['score']))
        o.position.x, o.position.y, o.position.z = map(float, obj['p'])
        o.size.x, o.size.y, o.size.z = map(float, obj['size'])
        array.observations.append(o)
    return array


def observations(objects, stamp_ns, frame, emulate, R_bc, K, image):
    """Injected objects -> ObservationArray (backend 'sim'); zed runs the boat's conversion."""
    if emulate == 'zed':
        from mhseals_nav.zed_backend import convert_objects
        array = convert_objects(zed_packet(stamp_ns, frame, objects, R_bc, K, image),
                                image, 'world')
    else:
        array = plain_array(stamp_ns, frame, objects)
    array.backend = 'sim'
    return array


def pinhole(image, hfov_deg):
    """(fx, fy, cx, cy) of a square-pixel camera with this image size and horizontal FOV."""
    width, height = image
    f = width / 2 / math.tan(math.radians(hfov_deg) / 2)
    return f, f, width / 2, height / 2


class SimBackend(Node):
    def __init__(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo
        from tf2_ros import Buffer, TransformListener
        from vision_msgs.msg import Detection3DArray
        from mhseals_nav.msg import ObservationArray
        from mhseals_nav.utils.live_params import LiveParams, finite, one_of, positive
        super().__init__('sim_backend', automatically_declare_parameters_from_overrides=True)
        self.cfg = nest({n: p.value for n, p in self.get_parameters_by_prefix('').items()
                         if n != 'use_sim_time'})
        cfg = self.cfg
        rules = {f'inject.{k}': finite for k in INJECT_KEYS + NOISE_KEYS}
        self.rng = np.random.default_rng(int(cfg['inject']['seed']))
        self.depth_state = {}
        rules.update(emulate=one_of('plain', 'zed'), max_detection_age=positive,
                     case=one_of('none', *cfg['cases']['names']))
        self.live = LiveParams(self, rules, self.apply)
        self.elements = []
        try:
            truth = json.loads(open(cfg['course_truth']).read())
            self.elements = [e for e in truth['elements'] if e.get('label')]
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().warning(f'no course truth ({error}): emulate zed runs as plain')
        self.case_start = self.now()
        self.apply(self.live.values, {})
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.T_bc = None
        self.K = pinhole((cfg['image']['width'], cfg['image']['height']), cfg['fov']['hfov_deg'])
        self.image = (cfg['image']['width'], cfg['image']['height'])
        self.odom = []  # (t, pose_wb, yaw_rate), newest last, ~2 s
        self.last_stamp, self.frame, self.reported = -1, '', None
        from mhseals_nav import perception_status
        self.status, self.stage = perception_status.status, perception_status.StageClock()
        self.Diag = DiagnosticArray
        self.pub = self.create_publisher(
            ObservationArray, '/perception/observations',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_pub = self.create_publisher(DiagnosticArray, '/perception/status', 1)
        self.create_timer(1.0, self.report)
        self.create_subscription(Odometry, cfg['truth_odom_topic'], self.truth, 50)
        self.create_subscription(CameraInfo, cfg['camera_info_topic'], self.camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, cfg['detections_topic'], self.detections,
                                 qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def apply(self, values, changed):
        from mhseals_nav.yaml_params import merge
        merge(self.cfg, nest(values))     # deep: inject.* keys without live rules stay
        if 'case' in changed:
            self.case_start = self.now()
        if self.cfg['emulate'] == 'zed' and not self.elements:
            self.get_logger().warning('emulate zed without course truth: publishing plain')

    def camera_info(self, msg):
        if msg.k[0] > 0:
            self.K, self.image = (msg.k[0], msg.k[4], msg.k[2], msg.k[5]), (msg.width, msg.height)

    def truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = [o for o in self.odom if t - o[0] < 2.0] + [
            (t, (quat_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])),
             msg.twist.twist.angular.z)]

    def truth_at(self, t):
        near = min(self.odom, key=lambda o: abs(o[0] - t), default=None)
        return near if near and abs(near[0] - t) <= 0.1 else None

    def base_to_camera(self, frame):
        if self.T_bc is None:
            from rclpy.time import Time
            from tf2_ros import TransformException
            try:
                tf = self.tf.lookup_transform(self.cfg['base_frame'], frame, Time()).transform
            except TransformException:
                return None
            r, t = tf.rotation, tf.translation
            self.T_bc = quat_matrix(r.x, r.y, r.z, r.w), np.array([t.x, t.y, t.z])
        return self.T_bc

    def detections(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = self.now() - stamp * 1e-9
        if stamp <= self.last_stamp:
            return self.stage.drop('reorder')
        if not -.1 <= age <= self.cfg['max_detection_age']:
            return self.stage.drop('stale')
        T_bc = self.base_to_camera(msg.header.frame_id)
        if T_bc is None:
            self.get_logger().warning(f'no TF {self.cfg["base_frame"]} -> '
                                      f'{msg.header.frame_id} yet', throttle_duration_sec=5.0)
            return self.stage.drop('malformed')
        self.last_stamp, self.frame = stamp, msg.header.frame_id
        try:
            dets = [(d.results[0].hypothesis.class_id, d.results[0].hypothesis.score,
                     np.array([d.bbox.center.position.x, d.bbox.center.position.y,
                               d.bbox.center.position.z]),
                     np.array([d.bbox.size.x, d.bbox.size.y, d.bbox.size.z]))
                    for d in msg.detections]
        except IndexError:
            return self.stage.drop('malformed')
        truth = self.truth_at(stamp * 1e-9)
        cfg, inj = self.cfg, self.injection()
        if inj != self.reported:  # phase boundaries reach the bag at frame resolution
            self.report()
        objects = scene(dets, stamp * 1e-9, truth and truth[1], *T_bc, cfg, self.elements)
        objects = inject(objects, *T_bc, heading_error(inj, truth[2] if truth else 0.0),
                         math.radians(inj['extrinsic_yaw_deg']),
                         (math.radians(inj['extrinsic_roll_deg']),
                          math.radians(inj['extrinsic_pitch_deg'])),
                         cfg['inject']['extrinsic_xyz_m'])
        objects = noisy(objects, T_bc[0], self.K, cfg['inject'], self.depth_state, self.rng,
                        stamp * 1e-9)
        emulate = cfg['emulate'] if self.elements else 'plain'
        array = observations(objects, stamp + int(inj['stamp_offset_s'] * 1e9),
                             msg.header.frame_id, emulate, T_bc[0], self.K, self.image)
        array.processing_latency = max(self.now() - stamp * 1e-9, 0.0)
        self.pub.publish(array)
        self.stage.accept(monotonic(), age, len(array.observations))

    def injection(self):
        return injection(self.cfg, self.cfg['case'], self.now() - self.case_start)

    def report(self):
        """1 Hz backend stage, plus the inject stage (also at once after a param set)."""
        cfg = self.cfg
        emulate = cfg['emulate'] if self.elements else 'plain'
        values = self.stage.values(monotonic())
        fov = cfg['fov']
        values.update(hfov_deg=f'{fov["hfov_deg"]:g}', range_m=f'{fov["range_m"]:g}',
                      frame=self.frame, ids=str(emulate == 'zed').lower(),
                      velocity='world' if emulate == 'zed' else 'none')
        inj = self.injection()
        echo = {'case': cfg['case'], 'emulate': emulate,
                'elapsed_s': f'{self.now() - self.case_start:.1f}',
                **{k: f'{v:g}' for k, v in inj.items()}}
        array = self.Diag()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self.status('backend', gethostname(), values, source='sim'),
                        self.status('inject', gethostname(), echo, source='sim')]
        self.status_pub.publish(array)
        self.reported = inj


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = SimBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
