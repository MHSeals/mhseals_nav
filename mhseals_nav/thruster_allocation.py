import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode, ParamSetV2, CommandLong
from rcl_interfaces.msg import ParameterValue, ParameterType

from geometry_msgs.msg import Twist


class ThrusterHardware(Node):

    def __init__(self, thruster_configs, max_retries=5):
        super().__init__('thruster_hardware')

        self.thruster_configs = thruster_configs
        self.max_retries = max_retries
        self.is_active = False
        self.is_armed = False
        self.original_params = {}
        self._rearm_cooldown = False

        self.current_channels = [0] * 18
        for cfg in self.thruster_configs.values():
            self.current_channels[cfg['channel'] - 1] = cfg.get('neutral_pwm', 1500)

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

        # Track arming state
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            qos
        )

        # Continuous override publish loop at 20 Hz (well within RC_OVERRIDE_TIME timeout)
        self.create_timer(0.05, self._override_timer_callback)

        # Persistent CommandLong client (reused for aux functions and arming)
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # MAVROS services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(ParamSetV2, '/mavros/param/set')

        self.get_logger().info("Waiting for MAVROS services...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for /mavros/set_mode")
        while not self.param_set_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("Interrupted while waiting for /mavros/param/set")
        self.get_logger().info("MAVROS services ready.")

    def set_param(self, name: str, value: int) -> bool:
        if not rclpy.ok():
            self.get_logger().error(
                f"set_param('{name}', {value}): rclpy not OK — node may be shutting down, skipping"
            )
            return False

        self.get_logger().debug(
            f"set_param: requesting param_id='{name}' integer_value={value} "
            f"via /mavros/param/set"
        )

        request = ParamSetV2.Request()
        request.param_id = name

        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = value

        request.value = parameter_value

        try:
            future = self.param_set_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as exc:
            self.get_logger().error(
                f"set_param('{name}', {value}): exception during async call: "
                f"{type(exc).__name__}: {exc}"
            )
            return False

        if not future.done():
            self.get_logger().error(
                f"set_param('{name}', {value}): timed out waiting for ParamSetV2 response "
                f"after 1.0s — MAVROS may not be running or /mavros/param/set is unresponsive"
            )
            return False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"set_param('{name}', {value}): exception retrieving result: "
                f"{type(exc).__name__}: {exc}"
            )
            return False

        if result is None:
            self.get_logger().error(
                f"set_param('{name}', {value}): ParamSetV2 service returned None — "
                f"service may be unavailable, crashed, or rejected the request"
            )
            return False

        # Extract the value the FCU confirmed back (may differ from what was requested)
        confirmed_int = getattr(result.value, 'integer_value', None)
        confirmed_real = getattr(result.value, 'double_value', None)

        if result.success:
            self.get_logger().debug(
                f"set_param('{name}'): success — requested={value}, "
                f"FCU confirmed integer_value={confirmed_int}, double_value={confirmed_real}"
            )
            if confirmed_int is not None and confirmed_int != value:
                self.get_logger().warn(
                    f"set_param('{name}'): FCU confirmed integer_value={confirmed_int} "
                    f"differs from requested value={value}"
                )
            return True

        self.get_logger().error(
            f"set_param('{name}'): FCU rejected parameter — "
            f"requested integer_value={value}, "
            f"FCU returned integer_value={confirmed_int}, double_value={confirmed_real}, "
            f"success={result.success}"
        )
        return False

    def send_aux_function(self, function_id: int, switch_pos: int) -> bool:
        """
        Sends MAV_CMD_DO_AUX_FUNCTION (218) via MAVROS CommandLong service.
        :param function_id: Aux function code (46 = RC Override Enable)
        :param switch_pos: 0 = Low (disable), 2 = High (enable)
        """
        if not self.command_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("COMMAND_LONG service not available")
            return False

        request = CommandLong.Request()
        request.command = 218  # MAV_CMD_DO_AUX_FUNCTION
        request.param1 = float(function_id)
        request.param2 = float(switch_pos)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        request.broadcast = False

        try:
            future = self.command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().error(
                f"send_aux_function({function_id}, {switch_pos}): exception: "
                f"{type(exc).__name__}: {exc}"
            )
            return False

        if not future.done():
            self.get_logger().error(
                f"send_aux_function({function_id}, {switch_pos}): timed out"
            )
            return False

        return True

    def force_arm(self) -> bool:
        """
        Blocking force-arm. Only call this OUTSIDE the spin loop (e.g. from activate()).
        Uses MAV_CMD_COMPONENT_ARM_DISARM (400) with param2=21196 to bypass pre-arm checks.
        """
        if not self.command_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("CommandLong service not available for force arm")
            return False

        request = self._make_arm_request()
        try:
            future = self.command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().error(f"force_arm: exception: {type(exc).__name__}: {exc}")
            return False

        if not future.done():
            self.get_logger().error("force_arm: timed out")
            return False

        self.get_logger().info("Force arm command sent")
        return True

    def _force_arm_async(self):
        """
        Non-blocking fire-and-forget arm. Safe to call from inside a callback/timer.
        """
        if not self.command_client.service_is_ready():
            self.get_logger().warn("CommandLong service not ready, skipping rearm")
            return
        self.command_client.call_async(self._make_arm_request())
        self.get_logger().info("Force arm request sent (async)")

    def _make_arm_request(self) -> CommandLong.Request:
        request = CommandLong.Request()
        request.command = 400    # MAV_CMD_COMPONENT_ARM_DISARM
        request.param1 = 1.0     # 1 = arm
        request.param2 = 21196.0 # bypass pre-arm checks
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        request.broadcast = False
        return request

    def store_original_params(self):
        # We trust default_value in config as original values
        for cfg in self.thruster_configs.values():
            self.original_params[cfg['param_name']] = cfg['default_value']

    def activate(self):
        self.get_logger().info("Enabling RC override on FCU...")

        success = self.send_aux_function(46, 2)  # 46 = RC Override Enable, 2 = High (on)
        if not success:
            self.get_logger().error("Failed to enable RC override")
            return False

        # Start sending neutral overrides NOW so the timer keeps them flowing
        # during every blocking set_param call below (spin_until_future_complete
        # processes timer callbacks, so overrides go out between each param set).
        self.stop_thrusters()
        self.is_active = True

        # Switch each SERVO from motor-mixer function to RC passthrough.
        # Neutral overrides are published continuously while this loop runs.
        for name, cfg in self.thruster_configs.items():
            pv = cfg['passthrough_value']
            self.get_logger().info(f"Setting {cfg['param_name']} to RC passthrough ({pv})...")
            if not self.set_param(cfg['param_name'], pv):
                self.get_logger().error(f"Failed to set {cfg['param_name']} to passthrough")
                self.is_active = False
                return False

        # Arm the FCU if it isn't already
        if not self.is_armed:
            self.get_logger().info("FCU not armed — force arming...")
            if not self.force_arm():
                self.get_logger().error("Failed to force arm FCU")
                self.is_active = False
                return False
        else:
            self.get_logger().info("FCU already armed")

        self.get_logger().info("Thrusters in RC passthrough mode")
        return True

    def deactivate(self):
        self.get_logger().info("Restoring thruster parameters...")

        # Reset to neutral and keep the timer publishing while we restore params.
        # is_active stays True so overrides keep flowing during each set_param call.
        self.stop_thrusters()
        self.send_aux_function(46, 0)  # Disable RC override

        # Restore SERVO functions back to motor-mixer values.
        # Neutral overrides keep flowing during this loop.
        for name, value in self.original_params.items():
            self.get_logger().info(f"Restoring {name} to motor function {value}...")
            success = self.set_param(name, value)
            if not success:
                self.get_logger().error(f"Failed restoring {name}")

        # Only now stop the override loop
        self.is_active = False
        self.get_logger().info("Thruster parameters restored.")
        return True

    def _reset_rearm_cooldown(self):
        self._rearm_cooldown = False

    def _state_callback(self, msg: State):
        self.is_armed = msg.armed

    def _override_timer_callback(self):
        """Publishes the current channel state at 20 Hz to keep the override alive."""
        if not self.is_active or not rclpy.ok():
            return

        # Re-arm if the FCU disarmed while we are active
        if not self.is_armed:
            if not self._rearm_cooldown:
                self._rearm_cooldown = True
                self.get_logger().warn("FCU is not armed while active — attempting force arm...")
                self._force_arm_async()
                # Reset cooldown after 3 s to allow a retry if still disarmed
                self.create_timer(3.0, self._reset_rearm_cooldown)
            return  # Skip this publish cycle; wait for state callback to confirm arming
        else:
            self._rearm_cooldown = False  # armed again, clear cooldown

        try:
            msg = OverrideRCIn()
            msg.channels = list(self.current_channels)
            self.override_pub.publish(msg)
        except Exception:
            pass

    def stop_thrusters(self):
        """Reset all thruster channels to neutral in the current state."""
        for cfg in self.thruster_configs.values():
            self.current_channels[cfg['channel'] - 1] = cfg.get('neutral_pwm', 1500)

    def cmd_vel_callback(self, msg: Twist):
        if not self.is_active:
            return

        surge = msg.linear.x
        sway = msg.linear.y
        yaw = msg.angular.z

        pwm = {
            'thruster_front_left':  1500 + surge*400 + sway*400 + yaw*200, # back left
            'thruster_front_right': 1500 + surge*400 - sway*400 - yaw*200, # front left
            'thruster_back_left':   1500 - surge*400 - sway*400 + yaw*200, # back right
            'thruster_back_right':  1500 - surge*400 + sway*400 - yaw*200, # front right
        }

        for name, cfg in self.thruster_configs.items():
            value = int(max(1100, min(1900, pwm[name])))
            self.current_channels[cfg['channel'] - 1] = value

    def set_manual_mode(self):
        if not rclpy.ok():
            self.get_logger().error(
                "set_manual_mode called but rclpy is not OK — node may be shutting down"
            )
            return False

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "MANUAL"

        self.get_logger().debug(
            f"Sending SetMode request: base_mode={request.base_mode}, "
            f"custom_mode='{request.custom_mode}' to /mavros/set_mode"
        )

        try:
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as exc:
            self.get_logger().error(
                f"Exception while calling /mavros/set_mode: {type(exc).__name__}: {exc}"
            )
            return False

        if not future.done():
            self.get_logger().error(
                "Timed out waiting for SetMode response (future not done after 1.0s) — "
                "MAVROS may not be running or /mavros/set_mode is unresponsive"
            )
            return False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Exception retrieving SetMode result: {type(exc).__name__}: {exc}"
            )
            return False

        if result is None:
            self.get_logger().error(
                "SetMode service call returned None — "
                "service may be unavailable, crashed, or rejected the request"
            )
            return False

        if result.mode_sent:
            self.get_logger().info(
                f"Mode set to MANUAL successfully "
                f"(base_mode={request.base_mode}, custom_mode='{request.custom_mode}')"
            )
            return True

        self.get_logger().error(
            f"Failed to set MANUAL mode — "
            f"mode_sent={result.mode_sent}, "
            f"requested base_mode={request.base_mode}, custom_mode='{request.custom_mode}'. "
            f"ArduPilot may have rejected the mode change (check vehicle state/arming status)."
        )
        return False

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    thrusters = {
        # default_value:      motor function (omni mixer) - used during normal ArduPilot operation
        # passthrough_value:  1 = RCPassThru for all servos; each SERVOn with function=1
        #                     automatically tracks its own-numbered RC channel (SERVO1→RC1, etc.)
        'thruster_back_left':  {'param_name': 'SERVO1_FUNCTION', 'default_value': 34, 'passthrough_value': 1, 'channel': 1},
        'thruster_front_left': {'param_name': 'SERVO2_FUNCTION', 'default_value': 35, 'passthrough_value': 1, 'channel': 2},
        'thruster_back_right':   {'param_name': 'SERVO3_FUNCTION', 'default_value': 33, 'passthrough_value': 1, 'channel': 3},
        'thruster_front_right':  {'param_name': 'SERVO4_FUNCTION', 'default_value': 36, 'passthrough_value': 1, 'channel': 4},
    }

    node = ThrusterHardware(thrusters)

    node.store_original_params()
    node.set_manual_mode()
    node.activate()

    restore_attempted = False

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Keyboard interrupt received, restoring thrusters and shutting down.")
            restore_attempted = True
            node.deactivate()
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok() and not restore_attempted:
            node.deactivate()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()