"""Backend observations -> world-frame tracks, track grid, course map and health (§5).

The filter runs in odom (continuous over a run); tracks are published in map. All
thresholds live in config/tracker.yaml; the logic is in track_filter, heading_check,
track_grid and course_map.
"""

from collections import deque
from datetime import datetime, timezone
import math
import os
from socket import gethostname
from time import monotonic

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray

from mhseals_nav import course_map, track_grid
from mhseals_nav.heading_check import HeadingCheck
from mhseals_nav.msg import ObservationArray, Track, TrackArray
from mhseals_nav.perception_status import StageClock, status
from mhseals_nav.track_filter import (
    MOTION_NAMES, MOTION_STATIC, SOURCE_LIVE, SOURCE_PRIOR, TrackFilter, stamp_of)
from mhseals_nav.tracker_markers import observation_markers, track_markers
from mhseals_nav.yaml_params import node_config

RELIABLE = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
OBSERVATIONS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)


def matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def unpack(transform):
    t = transform.transform
    return matrix(t.rotation), np.array([t.translation.x, t.translation.y, t.translation.z])


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker', automatically_declare_parameters_from_overrides=True)
        self.cfg = cfg = node_config(self, 'tracker.yaml')
        self.odom, self.map, self.base = cfg['tracking_frame'], cfg['map_frame'], cfg['base_frame']
        self.filter = TrackFilter(cfg)
        self.heading = HeadingCheck(cfg['heading_check'])
        self.heading_level = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pending = deque(maxlen=10)
        self.backend = 'none'
        self.clock = StageClock()
        self.seen = deque()                 # (t, class_id, score) for conf_/n_ keys
        self.latest_obs = None              # (header, rows) for detection markers
        self.marker_ids, self.marker_count = set(), 0
        self.grid_at = None
        self.clock_back = -math.inf
        self.last_tracks = None
        self.published_at = -math.inf

        self.create_subscription(ObservationArray, cfg['observations_topic'],
                                 self.on_observations, OBSERVATIONS)
        self.tracks_pub = self.create_publisher(TrackArray, '/perception/tracks', RELIABLE)
        self.objects_pub = self.create_publisher(Detection3DArray, '/objects/map', RELIABLE)
        self.track_markers_pub = self.create_publisher(MarkerArray, '/tracked_objects', RELIABLE)
        self.obs_markers_pub = self.create_publisher(
            MarkerArray, '/perception/detection_markers', RELIABLE)
        self.status_pub = self.create_publisher(DiagnosticArray, '/perception/status', RELIABLE)
        self.grid_pub = self.create_publisher(OccupancyGrid, cfg['grid']['topic'], LATCHED)
        self.create_timer(1.0 / cfg['rate_hz'], self.tick)
        self.create_timer(1.0 / cfg['grid']['rate_hz'], self.publish_grid)
        self.create_timer(1.0 / cfg['markers_hz'], self.publish_markers)
        self.create_timer(1.0 / cfg['status_hz'], self.publish_status)

        cm = cfg['course_map']
        sim = self.get_parameter('use_sim_time').value
        self.map_file = course_map.path(
            course_map.name_for(cm, sim, datetime.now(timezone.utc), os.getpid()))
        from robot_localization.srv import FromLL, ToLL
        self.ToLL, self.FromLL = ToLL, FromLL
        self.to_ll = self.create_client(ToLL, cm['to_ll_service'])
        self.from_ll = self.create_client(FromLL, cm['from_ll_service'])
        self.saved_ll = {}                  # track id -> course-map entry (lat/lon cached)
        self.prior_rows = {}                # prior track id -> its file entry
        self.loading = list(course_map.load(self.map_file, datetime.now(timezone.utc),
                                            cm['max_age_days'])) if cm['load'] else []
        # Entries not (yet) adopted as priors: kept in the file until they are, so a slow
        # or failed /fromLL never erases them.
        self.unadopted = {id(row): row for row in self.loading}
        self.had_rows = bool(self.loading)  # an empty map is written only after content
        self.warned_ll = False
        self.dirty = False
        self.create_timer(cm['save_period_s'], self.save_course_map)
        if self.loading:
            # navsat_transform answers /fromLL with zeros until its datum is set, and
            # publishes odometry/gps only after that, so the first fix gates the load.
            self.datum_sub = self.create_subscription(
                Odometry, cm['datum_topic'], self.datum_set, RELIABLE)
            self.load_timer = self.create_timer(1.0, self.load_course_map)

    # ------------------------------------------------------------------ TF
    def pose_at(self, t, frame):
        # Near sim time zero the filter's yaw-rate and camera-velocity spans reach before
        # t=0; rclpy Time rejects that, which killed the tracker and held costmap_stale.
        if t < 0.0:
            raise LookupError(f'no transform before time zero (t={t:.3f})')
        try:
            return unpack(self.tf.lookup_transform(
                self.odom, frame, Time(nanoseconds=int(t * 1e9))))
        except TransformException as error:
            raise LookupError(str(error)) from error

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def latest(self, target, source):
        try:
            return unpack(self.tf.lookup_transform(target, source, Time()))
        except TransformException as error:
            self.get_logger().warning(f'waiting for {target}<-{source}: {error}',
                                      throttle_duration_sec=5.0)
            return None

    # ---------------------------------------------------------- observations
    def on_observations(self, msg):
        self.backend = msg.backend or self.backend
        self.pending.append(msg)
        stepped = self.filter.last_stamp
        self.drain()
        # A fresh update goes out at once (still at most rate_hz), rather than waiting up
        # to one tick period for the timer: capture -> world-frame track loses 0-1/rate_hz.
        if self.cfg['publish_on_update'] and self.filter.last_stamp != stepped and \
                self.due():
            self.publish_tracks()

    def due(self):
        return monotonic() - self.published_at >= 0.9 / self.cfg['rate_hz']

    def drain(self):
        now = self.now()
        if now < self.clock_back:          # sim clock reset: start over
            self.filter = TrackFilter(self.cfg)
            self.pending.clear()
        self.clock_back = now
        for _ in range(len(self.pending)):
            msg = self.pending.popleft()
            stamp = stamp_of(msg.header)
            if now - stamp > self.cfg['max_detection_age']:
                self.clock.drop('stale')
                continue
            if stamp <= self.filter.last_stamp:
                self.clock.drop('reorder')
                continue
            if not self.filter.step(msg, now, self.pose_at):
                self.pending.append(msg)    # TF not there yet; retried until too old
                continue
            self.clock.accept(now, now - stamp)
            for sample in self.filter.heading_samples:
                self.heading.add(*sample)
            for ob in msg.observations:
                self.seen.append((now, ob.class_id, ob.score))
            self.latest_obs = (msg, now)
            self.dirty = True

    def tick(self):
        self.drain()
        self.filter.expire(self.now())
        if not self.cfg['publish_on_update'] or self.due():
            self.publish_tracks()

    # ------------------------------------------------------------ publishing
    def publish_tracks(self):
        transform = self.latest(self.map, self.odom)
        if transform is None:
            return
        self.published_at = monotonic()
        R, p = transform
        now_msg, now = self.get_clock().now().to_msg(), self.now()
        array = TrackArray()
        array.header.stamp, array.header.frame_id = now_msg, self.map
        objects = Detection3DArray(header=array.header)
        for track in self.filter.tracks:
            xyz = R @ np.array([track.x[0], track.x[1], track.z]) + p
            vel = R[:2, :2] @ track.x[2:]
            msg = Track(id=track.id, class_id=track.class_id, class_score=float(track.score),
                        backend_id=track.backend_id, motion=track.motion, source=track.source,
                        confidence=float(self.filter.confidence(track, now)),
                        speed_bound=float(track.speed_bound), hits=track.hits)
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = \
                map(float, xyz)
            msg.pose.pose.orientation.w = 1.0
            cov = np.zeros((6, 6))
            cov[:2, :2] = R[:2, :2] @ track.P[:2, :2] @ R[:2, :2].T
            msg.pose.covariance = cov.ravel().tolist()
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = map(float, vel)
            tcov = np.zeros((6, 6))
            tcov[:2, :2] = R[:2, :2] @ track.P[2:, 2:] @ R[:2, :2].T
            msg.twist.covariance = tcov.ravel().tolist()
            msg.size.x, msg.size.y, msg.size.z = map(float, track.size)
            msg.first_seen = Time(nanoseconds=int(track.first_seen * 1e9)).to_msg()
            msg.last_seen = Time(nanoseconds=int(track.last_seen * 1e9)).to_msg()
            array.tracks.append(msg)
            if track.source == SOURCE_LIVE:
                objects.detections.append(self.detection(objects.header, msg))
        self.tracks_pub.publish(array)
        self.objects_pub.publish(objects)
        self.last_tracks = array

    @staticmethod
    def detection(header, track):
        det = Detection3D(header=header, id=str(track.id))
        det.bbox.center = track.pose.pose
        det.bbox.size = track.size
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id, hyp.hypothesis.score = track.class_id, float(track.confidence)
        hyp.pose.pose = track.pose.pose
        det.results.append(hyp)
        return det

    def publish_grid(self):
        centre = self.latest(self.odom, self.base)
        if centre is None:
            return
        cfg = {**self.cfg['grid'], 'radius': self.cfg['radius']}
        now = self.now()
        cells = track_grid.rasterize(self.filter.tracks, centre[1][:2], cfg, now=now)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = self.get_clock().now().to_msg(), self.odom
        grid.info.resolution = float(cfg['resolution'])
        grid.info.width = grid.info.height = cells.shape[0]
        origin = track_grid.grid_origin(centre[1][:2], cfg)
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, origin)
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.ravel().tolist()
        self.grid_pub.publish(grid)
        self.grid_at = now

    def publish_markers(self):
        tracks = self.last_tracks
        if tracks is not None and self.track_markers_pub.get_subscription_count():
            rows = [(t.id, t.class_id, t.motion, t.source == SOURCE_PRIOR,
                     (t.pose.pose.position.x, t.pose.pose.position.y, t.pose.pose.position.z),
                     (t.twist.twist.linear.x, t.twist.twist.linear.y),
                     (t.size.x, t.size.y, t.size.z)) for t in tracks.tracks]
            array, self.marker_ids = track_markers(
                tracks.header, rows, self.marker_ids, 1 + int(2 / self.cfg['markers_hz']))
            self.track_markers_pub.publish(array)
        if self.latest_obs is not None and self.obs_markers_pub.get_subscription_count():
            msg, at = self.latest_obs
            rows = [] if self.now() - at > self.cfg['max_detection_age'] else [
                (o.class_id, o.score, (o.position.x, o.position.y, o.position.z),
                 (o.size.x, o.size.y, o.size.z)) for o in msg.observations]
            self.obs_markers_pub.publish(observation_markers(msg.header, rows, self.marker_count))
            self.marker_count = len(rows)

    def publish_status(self):
        now = self.now()
        window = 10.0
        while self.seen and now - self.seen[0][0] > window:
            self.seen.popleft()
        values = self.clock.values(now)
        tracks = self.filter.tracks
        for name in MOTION_NAMES:
            values[f'tracks_{name}'] = sum(t.motion == MOTION_NAMES.index(name) and
                                           t.source == SOURCE_LIVE for t in tracks)
        values['tracks_prior'] = sum(t.source == SOURCE_PRIOR for t in tracks)
        per_class = {}
        for _, cls, score in self.seen:
            per_class.setdefault(cls, []).append(score)
        for cls, scores in sorted(per_class.items()):
            values[f'conf_{cls}'] = f'{sum(scores) / len(scores):.2f}'
            values[f'n_{cls}'] = f'{len(scores) / window:.1f}'
        values['grid_age_s'] = 'inf' if self.grid_at is None else f'{now - self.grid_at:.2f}'
        assoc = self.filter.assoc
        values['id_assoc_share'] = f"{assoc['id'] / max(assoc['id'] + assoc['gate'], 1):.2f}"
        values.update({f'yaw_rate_{k}': v for k, v in self.filter.yaw_rate_from.items()})
        host = gethostname()
        result = self.heading.result(now)
        level = int(result['level'])
        if level != self.heading_level and self.heading_level is not None:
            lag, rho = result['lag_s'], result['rho']
            self.get_logger().warning(
                f'heading check level {level}: lag_s {lag:.3f} rho {rho:.2f}')
        self.heading_level = level
        heading = {k: (f'{v:.3f}' if isinstance(v, float) else str(v).lower())
                   for k, v in result.items() if k not in ('level', 'message')}
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            status('tracker', host, values, source=self.backend),
            status('heading', host, heading, level=level.to_bytes(1, 'little'),
                   message=result['message'], source=self.backend)]
        self.status_pub.publish(array)

    # ------------------------------------------------------------ course map
    def datum_set(self, _msg):
        self.destroy_subscription(self.datum_sub)
        self.datum_sub = None

    def load_course_map(self):
        if self.datum_sub is not None:
            self.get_logger().info('course map waiting for the navsat datum',
                                   throttle_duration_sec=30.0)
            return
        if not self.from_ll.service_is_ready():
            self.get_logger().info('course map waiting for fromLL', throttle_duration_sec=30.0)
            return
        self.load_timer.cancel()
        for row in self.loading:
            request = self.FromLL.Request()
            request.ll_point.latitude, request.ll_point.longitude = row['lat'], row['lon']
            self.from_ll.call_async(request).add_done_callback(
                lambda future, row=row: self.adopt_prior(row, future))
        self.get_logger().info(f'course map: {len(self.loading)} priors from {self.map_file}')
        self.loading = []

    def adopt_prior(self, row, future):
        try:
            result = future.result()
        except Exception as error:      # noqa: B902 - any service failure, same handling
            result, why = None, error
        else:
            why = 'no response'
        transform = self.latest(self.odom, self.map) if result is not None else None
        if transform is None:
            self.get_logger().warning(
                f"course map prior {row['class_id']} not adopted "
                f"({why if result is None else 'no map->odom'}); kept in the file")
            return
        point = result.map_point
        xy = (transform[0] @ np.array([point.x, point.y, 0.0]) + transform[1])[:2]
        track = self.filter.add_prior(row['class_id'], xy, row['sigma_m'], self.now())
        self.saved_ll[track.id] = row
        self.prior_rows[track.id] = row
        self.unadopted.pop(id(row), None)

    def eligible(self):
        """Return the STATIC buoy-group tracks to save: a boat that paused is no prior."""
        cm = self.cfg['course_map']
        return [t for t in self.filter.tracks if t.motion == MOTION_STATIC and
                t.group in cm['save_groups'] and (
                    (t.source == SOURCE_LIVE and t.hits >= cm['save_min_hits'])
                    or t.id in self.prior_rows)]

    def save_course_map(self):
        if not self.dirty:
            return
        if not self.to_ll.service_is_ready():
            if not self.warned_ll:
                self.get_logger().warning('course map not saved: toLL unavailable')
                self.warned_ll = True
            return
        transform = self.latest(self.map, self.odom)
        if transform is None:
            return
        self.dirty = False
        tracks = self.eligible()
        stamp = datetime.now(timezone.utc)
        for track in tracks:
            if track.source == SOURCE_PRIOR:
                continue                    # not re-observed: keep its file entry
            # The last measured position, not one predicted while out of view.
            seen = track.seen_xy
            xy = transform[0] @ np.array([seen[0], seen[1], 0.0]) + transform[1]
            request = self.ToLL.Request()
            request.map_point.x, request.map_point.y = float(xy[0]), float(xy[1])
            sigma = math.sqrt(max(float(np.linalg.eigvalsh(track.seen_P).max()), 0.0))
            row = {'class_id': track.class_id, 'sigma_m': round(sigma, 3), 'hits': track.hits,
                   'last_seen': course_map.iso(stamp)}
            self.to_ll.call_async(request).add_done_callback(
                lambda future, ident=track.id, row=row: self.converted(ident, row, future))
        self.write_course_map()

    def converted(self, ident, row, future):
        try:
            result = future.result()
        except Exception:               # noqa: B902 - keep the last good entry
            result = None
        if result is None:
            self.get_logger().warning('course map: toLL failed', throttle_duration_sec=30.0)
            return
        point = result.ll_point
        self.saved_ll[ident] = {**row, 'lat': point.latitude, 'lon': point.longitude}

    def write_course_map(self):
        alive = {t.id for t in self.eligible()}
        rows = [row for ident, row in self.saved_ll.items() if ident in alive and 'lat' in row]
        rows += list(self.unadopted.values())
        # Once the map had content, an empty one is news too (every buoy was dropped).
        if rows or self.had_rows:
            self.had_rows = self.had_rows or bool(rows)
            name = self.map_file.stem
            course_map.save(self.map_file, name, rows, datetime.now(timezone.utc))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_course_map()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
