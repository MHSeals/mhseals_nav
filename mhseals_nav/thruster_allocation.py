import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, ParamSetV2
from rcl_interfaces.msg import ParameterValue

from geometry_msgs.msg import Twist

import signal
import sys


class ThrusterHardware(Node):

    def __init__(self, thruster_configs, max_retries=5):
        super().__init__('thruster_hardware')

        self.thruster_configs = thruster_configs
        self.max_retries = max_retries
        self.is_active = False
        self.original_params = {}

        qos = QoSProfile(depth=10)

        # RC override publisher
        self.override_pub = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            qos
        )

        # cmd_vel subscriber
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        # MAVROS services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(ParamSetV2, '/mavros/param/set')

        self.get_logger().info("Waiting for MAVROS services...")
        self.set_mode_client.wait_for_service()
        self.param_set_client.wait_for_service()
        self.get_logger().info("MAVROS services ready.")

        signal.signal(signal.SIGINT, self._shutdown_handler)
        signal.signal(signal.SIGTERM, self._shutdown_handler)

    def set_param(self, name: str, value: int) -> bool:
        request = ParamSetV2.Request()
        request.param_id = name

        parameter_value = ParameterValue()
        parameter_value.integer_value = value

        request.value = parameter_value

        future = self.param_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success: # type: ignore
            return True
        return False

    def store_original_params(self):
        # We trust default_value in config as original values
        for cfg in self.thruster_configs.values():
            self.original_params[cfg['param_name']] = cfg['default_value']

    def activate(self):
        self.get_logger().info("Setting thrusters to RC passthrough...")

        for cfg in self.thruster_configs.values():
            success = self.set_param(cfg['param_name'], 1)
            if not success:
                self.get_logger().error(f"Failed setting {cfg['param_name']}")
                return False

        self.stop_thrusters()
        self.is_active = True
        self.get_logger().info("Thrusters in RC passthrough!")
        return True

    def deactivate(self):
        self.get_logger().info("Restoring thruster parameters...")

        self.is_active = False
        self.stop_thrusters()
        self.get_logger().info("Thrusters stopped!")

        for name, value in self.original_params.items():
            success = self.set_param(name, value)
            if not success:
                self.get_logger().error(f"Failed restoring {name}")
                return False

        self.get_logger().info("Thruster parameters restored.")
        return True

    def stop_thrusters(self):
        msg = OverrideRCIn()
        msg.channels = [0] * 18

        for cfg in self.thruster_configs.values():
            idx = cfg['channel'] - 1
            msg.channels[idx] = cfg.get('neutral_pwm', 1500)

        self.override_pub.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        if not self.is_active:
            return

        surge = msg.linear.x
        sway = msg.linear.y
        yaw = msg.angular.z

        pwm = {
            'thruster_front_left':  1500 + surge*400 + sway*400 + yaw*200,
            'thruster_front_right': 1500 + surge*400 - sway*400 - yaw*200,
            'thruster_back_left':   1500 - surge*400 - sway*400 + yaw*200,
            'thruster_back_right':  1500 - surge*400 + sway*400 - yaw*200,
        }

        out = OverrideRCIn()
        out.channels = [0] * 18

        for name, cfg in self.thruster_configs.items():
            value = int(max(1100, min(1900, pwm[name])))
            out.channels[cfg['channel'] - 1] = value

        self.override_pub.publish(out)

    def set_manual_mode(self):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "MANUAL"

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info("Mode set to MANUAL")
            return True

        self.get_logger().error("Failed to set MANUAL mode")
        return False

    def _shutdown_handler(self, signum, frame):
        self.get_logger().warn("Shutting down, restoring thrusters...")
        self.deactivate()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    thrusters = {
        'thruster_front_left':  {'param_name': 'SERVO1_FUNCTION', 'default_value': 36, 'channel': 1},
        'thruster_front_right': {'param_name': 'SERVO2_FUNCTION', 'default_value': 35, 'channel': 2},
        'thruster_back_left':   {'param_name': 'SERVO3_FUNCTION', 'default_value': 34, 'channel': 3},
        'thruster_back_right':  {'param_name': 'SERVO4_FUNCTION', 'default_value': 33, 'channel': 4},
    }

    node = ThrusterHardware(thrusters)

    node.store_original_params()
    node.set_manual_mode()
    node.activate()

    try:
        rclpy.spin(node)
    finally:
        node.deactivate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()