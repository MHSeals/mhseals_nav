"""Hold the FCU heading on the gyro while the compass is disturbed.

heading_guard turns /mavros/imu/data into /imu/raw. The FCU's yaw comes from the compass, and
thruster current near the compass bends the field: at high thrust the field strength swings far
from normal and the FCU heading jumps with no rotation (Blastoise, 11-24 and 11-27). Everything
that reads /imu/raw (ekf_global's yaw, navsat_transform's datum, the adapter's velocity
rotation, lidar fusion) would take those jumps.

The guard trusts the compass only while both hold:
  * field: the magnetometer's |B| is within [field_low, field_high] of its usual strength (a
    slow median of trusted samples, seeded at start);
  * jump: over the last ``window`` seconds the FCU heading moved with the gyro, to ``jump``.
Distrust is immediate; trust returns after ``recover`` seconds of both holding. A bent field
alone holds the heading at most ``hold`` seconds. Past that the guard follows the FCU again at
``slew``, still refusing jumps: a steel pool can bend the field for minutes, and the gyro is no
truer than the compass it last trusted, so its drift then costs more than it saves.

Output heading = gyro-integrated heading + offset. While trusted the offset is the FCU's, so a
healthy compass passes straight through. After a hold it moves at most ``slew`` rad/s, and a
disagreement over ``jump`` is taken only once it has stood ``accept`` seconds with no jump (the
FCU EKF has reconverged). While held, the yaw variance grows by ``drift`` rad/s of gyro drift.
Angles are radians, times seconds. The core has no ROS: bag replays and tests drive it directly.
"""
from collections import deque
import json
import math
import statistics

PARAMETERS = ('field_low', 'field_high', 'jump', 'window', 'recover', 'accept', 'slew', 'drift',
              'seed', 'field_time', 'hold')
MAG_STALE = 1.0     # s: a magnetometer sample older than this says nothing about the field now


def wrap(angle):
    """Return the angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class HeadingGuard:
    """Feed ``mag`` with field samples and ``imu`` with heading samples.

    ``imu`` returns the heading to publish. ``params`` maps PARAMETERS to values and is read on
    every sample, so a live parameter set applies at once.
    """

    def __init__(self, params):
        self.params = params
        self.gyro = None            # integrated gyro heading (unwrapped), rad
        self.last = None            # time of the last heading sample
        self.offset = None          # output heading - gyro heading
        self.history = deque()      # (time, FCU heading - gyro heading) over the jump window
        self.trusted, self.reason = True, 'starting'
        self.good_since = self.bad_since = None
        self.pending = None         # since when a disagreement over ``jump`` has stood
        self.settled = True         # converged while trusted: the FCU passes through exactly
        self.following = False      # held past ``hold`` of a bent field: following the FCU slowly
        self.field = None           # usual |B|
        self.seeding, self.started = [], None
        self.ratio, self.mag_time = None, None   # the last |B| / usual, and when

    # ---- inputs -----------------------------------------------------------------------------

    def mag(self, t, x, y, z):
        """Take a magnetometer sample; any unit, only its ratio to the usual strength counts."""
        strength = math.sqrt(x * x + y * y + z * z)
        if not math.isfinite(strength) or strength <= 0:
            return
        p = self.params
        if self.field is None:
            self.started = t if self.started is None else self.started
            self.seeding.append(strength)
            if t - self.started >= p['seed']:
                self.field = statistics.median(self.seeding)
                self.seeding = []
            self.ratio, self.mag_time = 1.0, t
            return
        dt = 0.0 if self.mag_time is None else max(0.0, t - self.mag_time)
        self.ratio, self.mag_time = strength / self.field, t
        if self.trusted and p['field_low'] <= self.ratio <= p['field_high']:
            # the usual strength drifts with heading (hard iron) and temperature: follow slowly
            self.field += (strength - self.field) * min(1.0, dt / p['field_time'])

    def imu(self, t, heading, rate):
        """Take the FCU's yaw (rad, ENU) and the gyro's yaw rate (rad/s).

        Return (heading to publish, extra yaw variance, or None while trusted).
        """
        p = self.params
        if self.gyro is None:
            self.gyro, self.offset, self.last = 0.0, wrap(heading), t
        dt = min(max(t - self.last, 0.0), 0.5)    # a gap or a clock step must not spin the gyro
        self.gyro += rate * dt
        self.last = t
        measured = wrap(heading - self.gyro)
        self.history.append((t, measured))
        while self.history and t - self.history[0][0] > p['window']:
            self.history.popleft()
        self._judge(t, measured)
        if self.trusted or self.following:
            self._follow(t, dt, measured)
        out = wrap(self.gyro + self.offset)
        if self.trusted:
            return out, None
        return out, (p['drift'] * min(t - self.bad_since, p['hold'])) ** 2

    # ---- decisions --------------------------------------------------------------------------

    def _judge(self, t, measured):
        p = self.params
        jumped = any(abs(wrap(measured - m)) > p['jump'] for _, m in self.history)
        fresh = (self.mag_time is not None and t - self.mag_time <= MAG_STALE
                 and self.field is not None)
        bent = fresh and not p['field_low'] <= self.ratio <= p['field_high']
        if jumped or bent:
            if self.trusted:
                self.bad_since = t
            # Exact pass-through only while trusted: while held or following, every change slews.
            self.trusted, self.good_since, self.settled = False, None, False
            if jumped:
                self.pending = None     # a large disagreement must stand ``accept`` s unjumped
            self.following = not jumped and t - self.bad_since >= p['hold']
            self.reason = ('jump' if jumped else
                           'field: following the FCU slowly' if self.following else 'field')
            return
        self.following = False
        self.good_since = t if self.good_since is None else self.good_since
        if not self.trusted:
            if t - self.good_since >= p['recover']:
                self.trusted, self.bad_since = True, None
            else:
                self.reason = 'recovering'
                return
        self.reason = 'ok' if fresh else 'ok (no magnetometer: jump check only)'

    def _follow(self, t, dt, measured):
        """Move the offset toward the FCU's.

        Settled, it is the FCU's as it is. After a hold it moves at most ``slew`` rad/s, and a
        disagreement over ``jump`` first waits ``accept`` s.
        """
        p = self.params
        if self.settled:
            self.offset = measured
            return
        error = wrap(measured - self.offset)
        if abs(error) > p['jump']:
            self.pending = t if self.pending is None else self.pending
            if t - self.pending < p['accept']:
                return
        else:
            self.pending = None
        step = p['slew'] * dt
        if abs(error) <= step:
            self.offset, self.settled, self.pending = measured, True, None
        else:
            self.offset = wrap(self.offset + math.copysign(step, error))

    def state(self):
        """Return what the guard is doing, for /localization/heading_guard."""
        held = (None if self.bad_since is None or self.last is None
                else round(self.last - self.bad_since, 1))
        return {'trusted': self.trusted, 'reason': self.reason,
                'field_ratio': None if self.ratio is None else round(self.ratio, 3),
                'offset_deg': None if self.offset is None else round(math.degrees(self.offset), 2),
                'held_s': held, 'pending': self.pending is not None}


# ---- ROS ------------------------------------------------------------------------------------

def yaw_of(q):
    """Return (roll, pitch, yaw) of a quaternion with x, y, z, w."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quaternion(roll, pitch, yaw):
    """Return (x, y, z, w) of roll, pitch, yaw (ZYX)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def guarded(message, guard, t, copy=True):
    """Return the Imu ``message`` with the guard's yaw.

    Its yaw variance grows while the compass is distrusted; roll, pitch, rates, accelerations
    and stamp are unchanged. ``copy=False`` edits the message itself: the node owns each one it
    receives, and a copy per sample costs time at 100 Hz.
    """
    roll, pitch, yaw = yaw_of(message.orientation)
    heading, extra = guard.imu(t, yaw, message.angular_velocity.z)
    if copy:
        from copy import deepcopy
        out = deepcopy(message)
    else:
        out = message
    o = out.orientation
    o.x, o.y, o.z, o.w = quaternion(roll, pitch, heading)
    if extra is not None:
        covariance = list(out.orientation_covariance)
        covariance[8] = max(covariance[8], 0.0) + extra
        out.orientation_covariance = covariance
    return out


def main(args=None):
    """Run the guard: /mavros/imu/data and /mavros/imu/mag in, /imu/raw out."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, MagneticField
    from std_msgs.msg import String

    from .odometry_configuration import resolve_odometry
    from .utils.live_params import LiveParams, positive, unit

    defaults = resolve_odometry().guard_parameters
    rclpy.init(args=args)
    node = Node('heading_guard')
    for name in PARAMETERS:
        node.declare_parameter(name, defaults[name])
    rules = {name: positive for name in PARAMETERS}
    rules['field_low'] = unit

    def bands(values):
        if not values['field_low'] < 1 < values['field_high']:
            raise ValueError('field_low < 1 < field_high')
    live = LiveParams(node, rules, joint=bands)
    guard = HeadingGuard(live)
    out = node.create_publisher(Imu, '/imu/raw', qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = node.create_publisher(String, '/localization/heading_guard', latched)
    last = {'state': None}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def receive(message):
        try:
            out.publish(guarded(message, guard, now(), copy=False))
        except Exception as error:     # never stall /imu/raw: the EKFs need every sample
            node.get_logger().warning(
                f'heading guard: {type(error).__name__}: {error}; passing the FCU attitude',
                throttle_duration_sec=5.)
            out.publish(message)

    def report():
        state = guard.state()
        if state == last['state']:
            return
        if last['state'] and state['trusted'] != last['state']['trusted']:
            node.get_logger().info(
                'compass trusted again' if state['trusted'] else
                f"compass distrusted ({state['reason']}): heading held on the gyro")
        status.publish(String(data=json.dumps(state)))
        last['state'] = state

    def receive_mag(message):
        f = message.magnetic_field
        guard.mag(now(), f.x, f.y, f.z)

    node.create_subscription(Imu, '/mavros/imu/data', receive, qos_profile_sensor_data)
    node.create_subscription(MagneticField, '/mavros/imu/mag', receive_mag,
                             qos_profile_sensor_data)
    node.create_timer(0.5, report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
