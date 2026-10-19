"""Adapt MAVROS GLOBAL_POSITION_INT odometry's ENU twist to body FLU.

MAVROS global_position/local labels its ENU velocity as base_link and uses
[-1, 0, ...] for unknown covariance. Neither is a valid EKF twist input.
Its pose already contains the FCU's ENU attitude; no compass offset belongs here.

``velocity_source: gps`` takes the velocity from the GPS receiver's own ENU
velocity (/mavros/global_position/raw/gps_vel) instead of the FCU EKF's. On
Blastoise (2026-10-18) the EKF's velocity grew to 1.9 m/s during a yaw in place
while the GPS read 0.1 m/s; the GPS velocity does not drift. Rotation into the
body still uses the FCU attitude, so the compass must be right.
"""
from copy import deepcopy
import math


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


def with_velocity(message, velocity):
    """The same odometry with its ENU velocity replaced and covariance unknown."""
    result = deepcopy(message)
    out = result.twist.twist.linear
    out.x, out.y, out.z = (float(v) for v in velocity)
    result.twist.covariance = [-1.] + [0.] * 35
    return result


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException

    from geometry_msgs.msg import TwistStamped

    rclpy.init(args=args)
    node = Node('mavros_odom_adapter')
    stddev = node.declare_parameter('velocity_stddev', .2).value
    source = node.declare_parameter('velocity_source', 'fcu').value
    max_age = node.declare_parameter('gps_velocity_max_age', .5).value
    if source not in ('fcu', 'gps'):
        raise ValueError('velocity_source must be fcu or gps')
    publisher = node.create_publisher(Odometry, '/odom/mavros', 10)
    gps = {}

    def receive(message):
        try:
            if source == 'gps':
                stamp = gps.get('stamp')
                age = (node.get_clock().now().nanoseconds * 1e-9 - stamp) if stamp else None
                if age is None or not -.1 <= age <= max_age:
                    node.get_logger().warning('GPS velocity is stale; not publishing',
                                              throttle_duration_sec=5.)
                    return
                message = with_velocity(message, gps['velocity'])
            publisher.publish(body_odometry(message, stddev))
        except ValueError as error:
            node.get_logger().warning(str(error), throttle_duration_sec=5.)

    def receive_gps(message):
        linear = message.twist.linear
        # Arrival time, not the header: MAVROS stamps GPS with the FCU's clock.
        gps.update(velocity=(linear.x, linear.y, linear.z),
                   stamp=node.get_clock().now().nanoseconds * 1e-9)

    if source == 'gps':
        node.create_subscription(TwistStamped, '/mavros/global_position/raw/gps_vel',
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
