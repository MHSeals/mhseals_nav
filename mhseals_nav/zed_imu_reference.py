"""The ZED 2i IMU and magnetometer as a second yaw reference for ekf_local.

Runs on the Orin beside the ZED wrapper. It republishes, best effort, for the EKF on the ODROID:

- ``/imu/zed``: the wrapper's IMU (/front/zed_node/imu/data) every ``rate_divisor``-th
  message, with the gyro variance raised to at least ``gyro_stddev_floor``. robot_localization
  estimates no gyro bias, and with equal variances a biased 200 Hz ZED gyro outweighs the FCU's
  bias-corrected 100 Hz rate (a 0.3 rad/s bias moved the fused rate by 0.27 in the bench). The
  floor keeps the FCU (MAVROS sigma 0.00035 rad/s) in charge while it streams, and the ZED
  carries the yaw rate through FCU stalls.
- ``/imu/zed_heading``: tilt-compensated magnetic heading as ENU yaw, only while
  ``heading_enabled`` is true. It stays false until the ZED magnetometer is calibrated with the
  ZED tools on the boat (owner bench list); uncalibrated, the camera's hard iron is unknown.

Both carry frame_id ``front_imu_link`` (camera body, FLU; the magnetometer shares its axes in the
ZED description), so the EKF takes the mount rotation from the boat URDF on its own host.
"""
import math

from mhseals_nav.utils.live_params import finite, LiveParams, positive


def boolean(value):
    if not isinstance(value, bool):
        raise ValueError('must be true or false')
    return value


def floored_covariance(covariance, floor_stddev):
    """A 3x3 covariance (9-list) with each diagonal raised to at least floor_stddev**2.

    A -1 first element (unknown) becomes the floor on the diagonal, zeros elsewhere.
    """
    floor = floor_stddev ** 2
    if covariance[0] < 0 or not all(math.isfinite(v) for v in covariance):
        covariance = [0.] * 9
    result = list(covariance)
    for i in (0, 4, 8):
        result[i] = max(result[i], floor)
    return result


def roll_pitch(q):
    """Roll and pitch (rad) of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1., min(1., 2 * (w * y - z * x))))
    return roll, pitch


def level_field(field, roll, pitch):
    """The body-frame (FLU) field rotated into the level frame that shares the body's yaw."""
    mx, my, mz = field
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    # R = Ry(pitch) Rx(roll): undo the tilt, keep the heading.
    y = cr * my - sr * mz
    z1 = sr * my + cr * mz
    x = cp * mx + sp * z1
    return x, y


def heading_yaw(field, orientation, declination):
    """ENU yaw (rad, CCW from east) of the body x axis from the magnetic field.

    field is the body-frame field vector, orientation the body's attitude quaternion (only its
    roll and pitch are used) and declination the local declination (rad, east positive).
    Magnetic north lies at atan2(y, x) in the level body frame, so the body heading is
    90 deg minus that, and true north is declination east of magnetic north.
    """
    roll, pitch = roll_pitch(orientation)
    x, y = level_field(field, roll, pitch)
    if math.hypot(x, y) < 1e-9:
        raise ValueError('no horizontal magnetic field')
    yaw = math.pi / 2 - math.atan2(y, x) - declination
    return math.atan2(math.sin(yaw), math.cos(yaw))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, MagneticField

    rclpy.init(args=args)
    node = Node('zed_imu_reference')
    camera = node.declare_parameter('camera_name', 'front').value
    frame = node.declare_parameter('frame_id', f'{camera}_imu_link').value
    divisor = node.declare_parameter('rate_divisor', 2).value
    node.declare_parameter('gyro_stddev_floor', 0.005)
    node.declare_parameter('heading_enabled', False)
    node.declare_parameter('heading_stddev', 0.1)
    node.declare_parameter('declination_deg', 0.0)
    if not isinstance(divisor, int) or divisor < 1:
        raise ValueError('rate_divisor must be a positive integer')
    live = LiveParams(node, {'gyro_stddev_floor': positive, 'heading_stddev': positive,
                             'heading_enabled': boolean, 'declination_deg': finite})
    imu_out = node.create_publisher(Imu, '/imu/zed', qos_profile_sensor_data)
    heading_out = node.create_publisher(Imu, '/imu/zed_heading', qos_profile_sensor_data)
    state = {'count': 0, 'orientation': None}

    def receive_imu(msg):
        q = msg.orientation
        state['orientation'] = (q.x, q.y, q.z, q.w)
        state['count'] += 1
        if state['count'] % divisor:
            return
        msg.header.frame_id = frame
        msg.angular_velocity_covariance = floored_covariance(
            list(msg.angular_velocity_covariance), live['gyro_stddev_floor'])
        imu_out.publish(msg)

    def receive_mag(msg):
        if not live['heading_enabled'] or state['orientation'] is None:
            return
        f = msg.magnetic_field
        try:
            yaw = heading_yaw((f.x, f.y, f.z), state['orientation'],
                              math.radians(live['declination_deg']))
        except ValueError as error:
            node.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame
        out.orientation.z, out.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        variance = live['heading_stddev'] ** 2
        # Roll and pitch are not measured here; two_d_mode ignores them.
        out.orientation_covariance = [1e6, 0., 0., 0., 1e6, 0., 0., 0., variance]
        out.angular_velocity_covariance[0] = -1.
        out.linear_acceleration_covariance[0] = -1.
        heading_out.publish(out)

    zed = f'/{camera}/zed_node'
    node.create_subscription(Imu, f'{zed}/imu/data', receive_imu, qos_profile_sensor_data)
    node.create_subscription(MagneticField, f'{zed}/imu/mag', receive_mag,
                             qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
