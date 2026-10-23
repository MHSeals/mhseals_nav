"""Adapt MAVROS GLOBAL_POSITION_INT odometry's ENU twist to body FLU.

MAVROS global_position/local labels its ENU velocity as base_link and uses
[-1, 0, ...] for unknown covariance. Neither is a valid EKF twist input.
Its pose already contains the FCU's ENU attitude; no compass offset belongs here.

``velocity_source: gps`` takes the velocity from the GPS receiver (GPS_RAW_INT
via /mavros/gpsstatus/gps1/raw) instead of the FCU EKF's. On Blastoise
(2026-10-18) the EKF's velocity grew to 1.9 m/s during a yaw in place while the
GPS read 0.1 m/s; the GPS velocity does not drift. Each receiver sample is
published once with its own accuracy as sigma. Rotation into the body still
uses the FCU attitude, so the compass must be right.
"""
from copy import deepcopy
import math

from mhseals_nav.utils.live_params import LiveParams, positive


def body_odometry(message, velocity_stddev):
    """Preserve pose/time, rotate linear velocity and covariance into child frame."""
    q = message.pose.pose.orientation
    values = [q.x, q.y, q.z, q.w]
    norm = math.sqrt(sum(v * v for v in values))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('FCU attitude is invalid')
    if not math.isfinite(velocity_stddev) or velocity_stddev <= 0:
        raise ValueError('velocity_stddev must be positive and finite')
    x, y, z, w = (v / norm for v in values)
    # R^T: world ENU -> body FLU, including the measured roll and pitch.
    rotation = ((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)),
                (2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)),
                (2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)))
    linear = message.twist.twist.linear
    velocity = (linear.x, linear.y, linear.z)
    if not all(math.isfinite(v) for v in velocity):
        raise ValueError('FCU velocity is non-finite')
    result = deepcopy(message)
    out = result.twist.twist.linear
    out.x, out.y, out.z = (sum(r[j] * velocity[j] for j in range(3)) for r in rotation)
    cov = message.twist.covariance
    variance = velocity_stddev ** 2
    known = (all(math.isfinite(cov[6*i+j]) for i in range(3) for j in range(3))
             and all(cov[7*i] > 0 for i in range(3)))
    world = [[cov[6*i+j] if known else (variance if i == j else 0.)
              for j in range(3)] for i in range(3)]
    result.twist.covariance = [0.] * 36
    for i in range(3):
        for j in range(3):
            result.twist.covariance[6*i+j] = sum(
                rotation[i][a] * world[a][b] * rotation[j][b]
                for a in range(3) for b in range(3))
    # GLOBAL_POSITION_INT does not measure angular velocity. EKF uses the IMU.
    for i in range(3, 6):
        result.twist.covariance[7*i] = 1e6
    return result


def gps_velocity(raw, min_stddev, nofix_stddev):
    """ENU velocity and sigma from a GPSRAW sample (vel cm/s, cog cdeg, vel_acc mm/s).

    Without a usable fix there is no measurement: a zero velocity with a wide
    sigma is a weak slow-boat prior rather than fake precision.
    """
    if raw.fix_type < 3 or 65535 in (raw.vel, raw.cog):
        return (0., 0., 0.), nofix_stddev
    speed, course = raw.vel / 100, math.radians(raw.cog / 100)
    return ((speed * math.sin(course), speed * math.cos(course), 0.),
            max(raw.vel_acc / 1000, min_stddev))


def with_velocity(message, velocity, stddev):
    """The same odometry with its ENU velocity and diagonal sigma replaced."""
    result = deepcopy(message)
    out = result.twist.twist.linear
    out.x, out.y, out.z = (float(v) for v in velocity)
    result.twist.covariance = [0.] * 36
    for i in range(3):
        result.twist.covariance[7*i] = stddev ** 2
    return result


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException

    from mavros_msgs.msg import GPSRAW

    rclpy.init(args=args)
    node = Node('mavros_odom_adapter')
    node.declare_parameter('velocity_stddev', .3)
    source = node.declare_parameter('velocity_source', 'gps').value
    node.declare_parameter('gps_velocity_max_age', .5)
    node.declare_parameter('gps_velocity_min_stddev', .05)
    node.declare_parameter('velocity_stddev_nofix', 1.)
    # The twist is the GPS antenna's: robot_localization removes the lever arm.
    gps_frame = node.declare_parameter('gps_frame', 'gps_link').value
    if source not in ('fcu', 'gps'):
        raise ValueError('velocity_source must be fcu or gps')
    # Tuning applies live; the source and frame decide subscriptions, so they do not.
    live = LiveParams(node, {name: positive for name in (
        'velocity_stddev', 'gps_velocity_max_age', 'gps_velocity_min_stddev',
        'velocity_stddev_nofix')}, restart_only=('velocity_source', 'gps_frame'))
    publisher = node.create_publisher(Odometry, '/odom/mavros', 10)
    gps = {}

    def receive(message):
        try:
            if source == 'gps':
                arrival = gps.get('arrival')
                age = (node.get_clock().now().nanoseconds * 1e-9 - arrival) if arrival else None
                if age is None or not -.1 <= age <= live['gps_velocity_max_age']:
                    node.get_logger().warning('GPS velocity is stale; not publishing',
                                              throttle_duration_sec=5.)
                    return
                if not gps.pop('new', False):
                    return  # fuse each receiver sample once, not at the 20 Hz FCU rate
                message = with_velocity(message, gps['velocity'], gps['stddev'])
                if gps['stamp_ok']:
                    message.header.stamp = gps['stamp']
            result = body_odometry(message, live['velocity_stddev'])
            result.child_frame_id = gps_frame if source == 'gps' else 'base_link'
            publisher.publish(result)
        except ValueError as error:
            node.get_logger().warning(str(error), throttle_duration_sec=5.)

    def receive_gps(raw):
        now = node.get_clock().now().nanoseconds * 1e-9
        stamp = raw.header.stamp
        velocity, sigma = gps_velocity(raw, live['gps_velocity_min_stddev'],
                                       live['velocity_stddev_nofix'])
        # Arrival time gates freshness since the header may carry FCU time; the
        # header is only the output stamp when it agrees with arrival.
        age = now - (stamp.sec + stamp.nanosec * 1e-9)
        gps.update(velocity=velocity, stddev=sigma, arrival=now, new=True, stamp=stamp,
                   stamp_ok=-.1 <= age <= live['gps_velocity_max_age'])

    if source == 'gps':
        node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw',
                                 receive_gps, qos_profile_sensor_data)

    node.create_subscription(Odometry, '/mavros/global_position/local', receive,
                             qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A callback may still publish while SIGTERM invalidates its DDS context.
        # Preserve real callback errors while the context is running.
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
