"""World-frame constant-velocity tracks from backend observations (perception-v2 §5.1-5.5).

Pure: no ROS I/O. ``pose_at(t, frame) -> (R 3x3, p 3)`` gives odom<-frame and raises
LookupError when TF cannot answer. Observations may be ``ObservationArray`` messages or
duck-types with the same field names.
"""

from collections import deque
from dataclasses import dataclass, field
from fnmatch import fnmatch
import math

import numpy as np

MOTION_UNKNOWN, MOTION_STATIC, MOTION_MOVING = 0, 1, 2
MOTION_NAMES = ('unknown', 'static', 'moving')
SOURCE_LIVE, SOURCE_PRIOR = 0, 1
VELOCITY_NONE, VELOCITY_RELATIVE, VELOCITY_WORLD = 0, 1, 2


@dataclass
class TrackState:
    id: int
    class_id: str
    group: str
    x: np.ndarray                  # [px, py, vx, vy] in the tracking frame
    P: np.ndarray                  # 4x4
    t: float                       # time x and P refer to
    first_seen: float
    last_seen: float
    score: float
    motion: int
    z: float = 0.0
    size: np.ndarray = field(default_factory=lambda: np.zeros(3))
    backend: str = ''
    backend_id: str = ''
    source: int = SOURCE_LIVE
    hits: int = 0
    confirmed: bool = False
    speed_bound: float = 0.0
    move_s: float = 0.0            # consecutive moving / static evidence (s)
    static_s: float = 0.0
    hit_times: list = field(default_factory=list)
    fov_miss_since: float = None
    anchor: np.ndarray = None
    anchor_t: float = 0.0
    seen_xy: np.ndarray = None     # position and covariance after the last measurement
    seen_P: np.ndarray = None

    def __post_init__(self):
        if self.seen_xy is None:
            self.seen_xy, self.seen_P = self.x[:2].copy(), self.P[:2, :2].copy()


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(R):
    return math.atan2(R[1][0], R[0][0])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def cv_model(dt, q):
    F = np.eye(4)
    F[0, 2] = F[1, 3] = dt
    Q = np.zeros((4, 4))
    Q[:2, :2] = np.eye(2) * dt ** 3 / 3
    Q[:2, 2:] = Q[2:, :2] = np.eye(2) * dt ** 2 / 2
    Q[2:, 2:] = np.eye(2) * dt
    return F, Q * q


def speed_bound(x, P, cfg, lag_mps=0.0):
    """k_sigma times the speed sigma along the velocity, plus the floor (§5.3).

    ``lag_mps`` is the velocity a stamp lag can fake right now (rho*sigma_lag*|dr/dt|);
    it adds to the filter's own sigma.
    """
    v, Pvv = x[2:], P[2:, 2:]
    speed = float(np.hypot(*v))
    if speed < 1e-6:
        sigma = math.sqrt(max(np.trace(Pvv) / 2, 0.0))
    else:
        u = v / speed
        sigma = math.sqrt(max(float(u @ Pvv @ u), 0.0))
    sigma = math.hypot(sigma, lag_mps)
    return cfg['k_sigma'] * sigma + cfg['floor_mps'], speed


class TrackFilter:
    """Predict, associate (id pass then global Mahalanobis gate), update, classify, expire."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.all = []               # candidates, live and prior tracks
        self.next_id = 1
        self.last_stamp = -math.inf
        self.heading_samples = []   # (t, a, r, track_id), cleared at the start of step()
        self.assoc = {'id': 0, 'gate': 0}
        self.yaw_rates = deque()    # (t, r, |r|, |dr/dt|) over meas.lag_window_s
        self.lag_speed = 0.0        # sigma_lag * max |dr/dt|: per metre of range, this frame
        self.yaw_rate_from = {'central': 0, 'backward': 0, 'none': 0}
        self._groups = {}

    @property
    def tracks(self):
        return [t for t in self.all if t.confirmed]

    def group(self, class_id):
        if class_id not in self._groups:
            groups = self.cfg['class_groups']
            name = next((g for g, spec in groups.items() if g != 'unknown' and any(
                fnmatch(class_id, p) for p in spec.get('match', []))), 'unknown')
            self._groups[class_id] = name
        return self._groups[class_id]

    def _spec(self, track_or_class):
        name = track_or_class.group if isinstance(track_or_class, TrackState) else \
            self.group(track_or_class)
        return self.cfg['class_groups'][name]

    def _prior_motion(self, class_id):
        return MOTION_NAMES.index(self._spec(class_id).get('prior', 'unknown'))

    # ---------------------------------------------------------------- model
    def predict(self, track, t):
        dt = t - track.t
        if dt <= 0:
            return
        F, Q = cv_model(dt, self._spec(track)['q'])
        track.x = F @ track.x
        track.P = F @ track.P @ F.T + Q
        track.t = t

    def predicted(self, track, t):
        """(x, P) at ``t`` without changing the track."""
        dt = max(t - track.t, 0.0)
        F, Q = cv_model(dt, self._spec(track)['q'])
        return F @ track.x, F @ track.P @ F.T + Q

    def confidence(self, track, now):
        motion = MOTION_NAMES[track.motion]
        value = track.score * math.exp(-max(now - track.last_seen, 0.0) / self.cfg['tau'][motion])
        if track.source == SOURCE_PRIOR:
            value = min(self.cfg['course_map']['prior_confidence'], value)
        return value

    # ---------------------------------------------------------- measurement
    def _lag_rates(self, t, yaw_rate):
        """(max |r|, max |dr/dt|) over the last ``lag_window_s`` (§5.1 lag term).

        A stamp lag tau offsets a static object across the bearing by rho*tau*r, so the
        velocity it fakes is rho*tau*dr/dt: largest while r reverses through zero, where
        |r| itself is near zero. The window holds both through the reversal and for as
        long as the filter remembers the velocity the reversal faked.
        """
        meas, h = self.cfg['meas'], self.yaw_rates
        if yaw_rate is not None:
            old = next(((s, r) for s, r, _, _ in h if t - s <= meas['yaw_accel_dt']), None)
            accel = abs(yaw_rate - old[1]) / (t - old[0]) if old and t > old[0] else 0.0
            h.append((t, yaw_rate, abs(yaw_rate), accel))
        while h and t - h[0][0] > meas['lag_window_s']:
            h.popleft()
        return (max((a for _, _, a, _ in h), default=0.0),
                max((d for _, _, _, d in h), default=0.0))

    def _measurement(self, ob, R, p, c, v_cam, yaw_rate):
        """(z, Rz, kind) in the tracking frame; z has 4 entries when velocity is used."""
        meas = self.cfg['meas']
        local = np.array([ob.position.x, ob.position.y, ob.position.z])
        world = R @ local + p
        offset = world[:2] - c[:2]
        rho = max(float(np.hypot(*offset)), 1e-3)
        cov = np.array(ob.position_covariance, dtype=float).reshape(3, 3)
        if np.any(cov):
            Rz = (R @ cov @ R.T)[:2, :2]
        else:
            Rz = np.eye(2) * max(meas['min_sigma_m'], meas['range_frac'] * rho) ** 2
        # Stereo depth error grows with range squared (sigma_z = z^2 sigma_d / (f B)), and the
        # ZED's own object covariance does not carry it: on 2026-10-25 the same buoy came back
        # 0.6-2.5 m deeper or nearer at 6-10 m against a reported 0.15 m, and started new ids.
        u = offset / rho
        Rz = Rz + (meas['depth_k'] * rho ** 2) ** 2 * np.outer(u, u)
        # Heading noise and stamp lag sweep a static object across the bearing.
        n = np.array([-offset[1], offset[0]]) / rho
        tangential = (rho * math.radians(meas['heading_sigma_deg'])) ** 2 + \
            (rho * meas['lag_sigma_s'] * abs(yaw_rate)) ** 2
        Rz = Rz + tangential * np.outer(n, n)
        kind = ob.velocity_kind
        velocity = None
        if kind in (VELOCITY_RELATIVE, VELOCITY_WORLD):
            v = R @ np.array([ob.velocity.x, ob.velocity.y, ob.velocity.z])
            if kind == VELOCITY_RELATIVE:
                v = v + v_cam
            vcov = np.array(ob.velocity_covariance, dtype=float).reshape(3, 3)
            Rv = (R @ vcov @ R.T)[:2, :2] if np.any(vcov) else \
                np.eye(2) * self.cfg['velocity_sigma'] ** 2
            velocity = (v[:2], Rv)
        return world, Rz, velocity

    def _innovation(self, track, z, Rz, velocity):
        use_v = velocity is not None and self.cfg['use_backend_velocity']
        H = np.eye(4) if use_v else np.eye(2, 4)
        zz = np.concatenate([z[:2], velocity[0]]) if use_v else z[:2]
        Rm = Rz
        if use_v:
            Rm = np.zeros((4, 4))
            Rm[:2, :2], Rm[2:, 2:] = Rz, velocity[1]
        nu = zz - H @ track.x
        S = H @ track.P @ H.T + Rm
        d2 = float(nu @ np.linalg.solve(S, nu))
        return d2, H, nu, S, use_v

    def _compatible(self, track, class_id):
        if track.class_id == class_id:
            return True
        return bool(self.cfg['group_match']) and track.group == self.group(class_id) \
            and track.group != 'unknown'

    # ---------------------------------------------------------------- step
    def step(self, obs, now, pose_at):
        """Fold one ObservationArray in; returns False when it was dropped or TF missing."""
        self.heading_samples = []
        t = stamp_of(obs.header)
        if t <= self.last_stamp:
            return False
        frame = obs.header.frame_id
        try:
            R, p = (np.asarray(a, dtype=float) for a in pose_at(t, frame))
        except LookupError:
            return False
        meas = self.cfg['meas']
        yaw_rate = v_cam = None
        # Central difference over [t-dt, t+dt]. A fresh frame usually arrives before TF
        # reaches t+dt, so the backward difference over [t-2dt, t] stands in rather than
        # leaving the lag term and the heading check without this frame's yaw rate.
        dt = self.cfg['heading_check']['yaw_dt']
        base = self.cfg.get('base_frame', 'base_link')
        spans = [(-dt, dt, 'central')]
        if meas.get('yaw_rate_backward', False):
            spans.append((-2 * dt, 0.0, 'backward'))
        for lo, hi, how in spans:
            try:
                Ra, _ = pose_at(t + lo, base)
                Rb, _ = pose_at(t + hi, base)
            except LookupError:
                continue
            yaw_rate = wrap(yaw_of(Rb) - yaw_of(Ra)) / (hi - lo)
            self.yaw_rate_from[how] += 1
            break
        else:
            self.yaw_rate_from['none'] += 1
        try:
            dt = meas['cam_vel_dt']
            _, pa = pose_at(t - dt, frame)
            _, pb = pose_at(t + dt, frame)
            v_cam = (np.asarray(pb, dtype=float) - np.asarray(pa, dtype=float)) / (2 * dt)
        except LookupError:
            v_cam = np.zeros(3)
        self.last_stamp = t
        # A coasting STATIC track does not move: its velocity is a noise estimate, and over
        # its 900 s lifetime it would carry the disc and the course map off the buoy.
        held = [(track, track.x[:2].copy()) for track in self.all
                if track.confirmed and track.motion == MOTION_STATIC]
        for track in self.all:
            self.predict(track, t)

        rate, accel = self._lag_rates(t, yaw_rate)
        self.lag_speed = meas['lag_sigma_s'] * accel
        rows = [(ob, *self._measurement(ob, R, p, p, v_cam, rate))
                for ob in obs.observations]
        live = list(self.all)
        matched_obs, matched_tracks, pairs = set(), set(), []
        # Id pass: the backend's own id wins over a nearer track without it. Several tracks
        # can carry the id (a candidate restarted from it), so the nearest one in gate wins.
        for i, (ob, z, Rz, vel) in enumerate(rows):
            if not ob.backend_id:
                continue
            best = None
            for k, track in enumerate(live):
                if k in matched_tracks or track.backend != obs.backend or \
                        track.backend_id != ob.backend_id or track.class_id != ob.class_id:
                    continue
                gate = self._innovation(track, z, Rz, vel)
                if gate[0] < self.cfg['gate_chi2_id'] and (best is None or gate[0] < best[1][0]):
                    best = (k, gate)
            if best is not None:
                pairs.append((i, best[0], best[1], 'id'))
                matched_obs.add(i)
                matched_tracks.add(best[0])
        # Gate pass: global order of increasing d2.
        candidates, d2s = [], []
        for i, (ob, z, Rz, vel) in enumerate(rows):
            if i in matched_obs:
                continue
            for k, track in enumerate(live):
                if k in matched_tracks or not self._compatible(track, ob.class_id):
                    continue
                gate = self._innovation(track, z, Rz, vel)
                d2s.append((i, k, gate[0]))
                limit = self.cfg['gate_chi2_v'] if gate[4] else self.cfg['gate_chi2']
                if gate[0] < limit:
                    candidates.append((gate[0], i, k, gate))
        for _, i, k, gate in sorted(candidates, key=lambda c: c[:3]):
            if i in matched_obs or k in matched_tracks:
                continue
            pairs.append((i, k, gate, 'gate'))
            matched_obs.add(i)
            matched_tracks.add(k)

        for i, k, gate, how in pairs:
            ob, z, Rz, _ = rows[i]
            self._update(live[k], ob, obs.backend, z, Rz, gate, t, p, yaw_rate)
            if live[k].confirmed:
                self.assoc[how] += 1
        updated = {id(live[k]) for k in matched_tracks}
        for track, xy in held:
            if id(track) not in updated:
                track.x[:2] = xy            # P keeps its predicted growth
        # A tail outlier of an existing track must not seed its duplicate. Only tracks left
        # unmatched count: one that took its own observation must not hide a neighbour.
        nearest = [math.inf] * len(rows)
        for i, k, d2 in d2s:
            if k not in matched_tracks:
                nearest[i] = min(nearest[i], d2)
        for i, (ob, z, Rz, vel) in enumerate(rows):
            if i not in matched_obs and nearest[i] >= self.cfg['candidate']['exclusion_chi2']:
                self._start(ob, obs.backend, z, Rz, vel, t)
        self._miss_in_fov(R, p, frame, live, matched_tracks, t)
        self.expire(max(now, t))
        return True

    def _update(self, track, ob, backend, z, Rz, gate, t, c, yaw_rate):
        d2, H, nu, S, _ = gate
        prior_xy = track.x[:2].copy()
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ nu
        track.P = (np.eye(4) - K @ H) @ track.P
        track.P = (track.P + track.P.T) / 2
        track.seen_xy, track.seen_P = track.x[:2].copy(), track.P[:2, :2].copy()
        alpha = self.cfg['score_alpha']
        track.score += alpha * (ob.score - track.score)
        size = np.array([ob.size.x, ob.size.y, ob.size.z])
        if np.any(size):
            track.size = size if not np.any(track.size) else \
                track.size + self.cfg['size_alpha'] * (size - track.size)
        track.z += self.cfg['size_alpha'] * (z[2] - track.z)
        track.hits += 1
        dt, track.last_seen = t - track.last_seen, t
        track.backend, track.backend_id = backend, ob.backend_id
        track.source = SOURCE_LIVE
        track.fov_miss_since = None
        if not track.confirmed:
            cand = self.cfg['candidate']
            track.hit_times = [h for h in track.hit_times if t - h <= cand['time']] + [t]
            if len(track.hit_times) >= cand['hits']:
                track.confirmed = True
                track.hit_times = []
            return
        self._classify(track, dt, Rz, c)
        check = self.cfg['heading_check']
        if track.motion != MOTION_STATIC:
            track.anchor = None
            return
        # The CV prediction follows a turn's sweep, so the residual is taken against a
        # slow anchor of the static position instead of the prior estimate.
        if track.anchor is None:
            track.anchor, track.anchor_t = prior_xy, t
        gain = 1 - math.exp(-(t - track.anchor_t) / check['anchor_tau_s'])
        track.anchor = track.anchor + gain * (z[:2] - track.anchor)
        track.anchor_t = t
        if track.hits >= check['min_hits'] and yaw_rate is not None:
            offset = track.anchor - c[:2]
            rho = float(np.hypot(*offset))
            if rho > 1e-3:
                n = np.array([-offset[1], offset[0]]) / rho
                a = float((z[:2] - track.anchor) @ n) / rho
                self.heading_samples.append((t, a, yaw_rate, track.id))

    def _displaced(self, track, Rz):
        """Return whether a STATIC track's estimate left its slow anchor beyond the noise.

        Velocity alone misses a slow mover: its speed can sit between the release and
        the bound for good. Displacement grows with time while the noise does not. Only
        groups with ``move_chi2`` use it: a moored buoy's heading error (bias that turns
        with the bearing, lag in turns) displaces it by metres at range.
        """
        chi2 = self._spec(track).get('move_chi2')
        if track.anchor is None or not chi2:
            return False
        delta = track.x[:2] - track.anchor
        S = Rz[:2, :2] + track.P[:2, :2]
        return float(delta @ np.linalg.solve(S, delta)) > chi2

    def _classify(self, track, dt, Rz, c):
        """Hysteresis in seconds of consecutive evidence, so it does not depend on the rate."""
        spec, cfg = self._spec(track), self.cfg['motion']
        rho = float(np.hypot(*(track.x[:2] - c[:2])))
        bound, speed = speed_bound(track.x, track.P, cfg, rho * self.lag_speed)
        track.speed_bound = bound
        step = min(max(dt, 0.0), cfg['max_step_s'])
        if speed > bound or self._displaced(track, Rz):
            track.move_s, track.static_s = track.move_s + step, 0.0
        elif speed < cfg['release_frac'] * bound and bound <= cfg['static_bound_max_mps']:
            # A young track's wide bound would call anything static.
            track.move_s, track.static_s = 0.0, track.static_s + step
        else:
            track.move_s = track.static_s = 0.0
        if track.motion == MOTION_UNKNOWN and self._prior_motion(track.class_id) == \
                MOTION_UNKNOWN and track.hits < cfg['n_unknown_min']:
            return
        eps = 1e-6
        if track.motion != MOTION_MOVING and track.move_s >= spec['move_s'] - eps:
            track.motion = MOTION_MOVING
        elif track.motion != MOTION_STATIC and track.static_s >= spec['static_s'] - eps:
            track.motion = MOTION_STATIC

    def _start(self, ob, backend, z, Rz, velocity, t):
        spec = self._spec(ob.class_id)
        x = np.zeros(4)
        x[:2] = z[:2]
        if velocity is not None:
            x[2:] = velocity[0]
        P = np.zeros((4, 4))
        P[:2, :2] = Rz
        P[2:, 2:] = np.eye(2) * spec['sigma_v0'] ** 2
        track = TrackState(
            id=self.next_id, class_id=ob.class_id, group=self.group(ob.class_id), x=x, P=P,
            t=t, first_seen=t, last_seen=t, score=float(ob.score),
            motion=self._prior_motion(ob.class_id), z=float(z[2]),
            size=np.array([ob.size.x, ob.size.y, ob.size.z]), backend=backend,
            backend_id=ob.backend_id, hits=1, hit_times=[t])
        self.next_id += 1
        track.confirmed = self.cfg['candidate']['hits'] <= 1
        self.all.append(track)

    def add_prior(self, class_id, xy, sigma_m, now):
        """Add a course-map entry as a SOURCE_PRIOR STATIC track (§5.6)."""
        cm = self.cfg['course_map']
        spec = self._spec(class_id)
        P = np.zeros((4, 4))
        P[:2, :2] = np.eye(2) * (sigma_m ** 2 + cm['prior_sigma_m'] ** 2)
        P[2:, 2:] = np.eye(2) * spec['sigma_v0'] ** 2
        track = TrackState(
            id=self.next_id, class_id=class_id, group=self.group(class_id),
            x=np.array([xy[0], xy[1], 0.0, 0.0]), P=P, t=now, first_seen=now, last_seen=now,
            score=cm['prior_confidence'], motion=MOTION_STATIC, source=SOURCE_PRIOR,
            confirmed=True)
        self.next_id += 1
        self.all.append(track)
        return track

    def in_fov(self, R, p, frame, xy):
        """Return whether xy is in the backend FOV and range; optical frames look along z."""
        fov = self.cfg['fov']
        forward = R[:, 2] if 'optical' in frame else R[:, 0]
        offset = np.asarray(xy) - p[:2]
        rho = float(np.hypot(*offset))
        if rho > fov['range_m'] or rho < 1e-3 or not np.any(forward[:2]):
            return False
        bearing = wrap(math.atan2(offset[1], offset[0]) - math.atan2(forward[1], forward[0]))
        return abs(bearing) <= math.radians(fov['hfov_deg']) / 2

    def _miss_in_fov(self, R, p, frame, live, matched, t):
        """Negative evidence: a STATIC track in view that nobody sees for a while is gone."""
        for k, track in enumerate(live):
            if k in matched or not track.confirmed or track.motion != MOTION_STATIC:
                continue
            if not self.in_fov(R, p, frame, track.x[:2]):
                track.fov_miss_since = None
            elif track.fov_miss_since is None:
                track.fov_miss_since = t

    def expire(self, now):
        cfg = self.cfg
        keep = []
        for track in self.all:
            age = now - track.last_seen
            if not track.confirmed:
                alive = age <= cfg['candidate']['timeout']
            else:
                alive = age <= cfg['timeout'][MOTION_NAMES[track.motion]] and (
                    track.fov_miss_since is None
                    or now - track.fov_miss_since <= cfg['miss_in_fov_s'])
            if alive:
                keep.append(track)
        self.all = keep
