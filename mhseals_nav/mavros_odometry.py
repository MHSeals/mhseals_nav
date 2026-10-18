"""Adapt MAVROS GLOBAL_POSITION_INT odometry's ENU twist to body FLU.

MAVROS global_position/local labels its ENU velocity as base_link and uses
[-1, 0, ...] for unknown covariance. Neither is a valid EKF twist input.
Its pose already contains the FCU's ENU attitude; no compass offset belongs here.
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


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = Node('mavros_odom_adapter')
    stddev = node.declare_parameter('velocity_stddev', .2).value
    publisher = node.create_publisher(Odometry, '/odom/mavros', 10)

    def receive(message):
        try:
            publisher.publish(body_odometry(message, stddev))
        except ValueError as error:
            node.get_logger().warning(str(error), throttle_duration_sec=5.)

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
