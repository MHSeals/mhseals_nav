"""The shape shared by the hold-only safety watchdogs (link, fence and FCU).

Each watches one condition. While it is bad and autonomy commands flow (an auto command
within command_window_s), the watchdog asks command_mux for a hold with its own cause and
token, and asks again every repeat_s, so a release while the condition is still bad is held
again. It never releases: the operator clears holds. Ages are receipt ages on the monotonic
clock, so a clock skew between hosts can neither cause nor hide a hold.
"""

import json
import secrets
import time


def fresh(at, now, limit):
    return at is not None and now - at <= limit


class HoldGate:
    """Decide when to (re)send a hold request; ``reason(now)`` is '' while all is well."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.command_at = None
        self.sent_at = None

    def reason(self, now):
        raise NotImplementedError

    def command(self, now):
        self.command_at = now

    def autonomy(self, now):
        return fresh(self.command_at, now, self.cfg['command_window_s'])

    def tick(self, now):
        """Return True when a hold request should go out now."""
        if not (self.reason(now) and self.autonomy(now)):
            return False
        if self.sent_at is not None and now - self.sent_at < self.cfg['repeat_s']:
            return False
        self.sent_at = now
        return True


def node_class(name, filename, logic, subscribe):
    """Build a watchdog node: ``logic(cfg)`` decides, ``subscribe(node)`` feeds ``node.logic``.

    Built lazily so the decision logic imports without rclpy. Keyword arguments of the
    returned class go to rclpy's Node (tests pass parameter_overrides).
    """
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    from mhseals_nav.yaml_params import node_config

    class HoldWatchdog(Node):
        def __init__(self, **kwargs):
            super().__init__(name, automatically_declare_parameters_from_overrides=True,
                             **kwargs)
            self.cfg = cfg = node_config(self, filename)
            self.logic = logic(cfg)
            self.token = secrets.token_hex(8)
            self.request = self.create_publisher(String, cfg['request_topic'], 10)
            self.create_subscription(Twist, cfg['command_topic'], self.on_command, 10)
            subscribe(self)
            self.create_timer(1.0 / cfg['rate_hz'], self.tick)
            self.was = None

        def on_command(self, _):
            self.logic.command(time.monotonic())
            self.tick()

        def tick(self):
            now = time.monotonic()
            why = self.logic.reason(now)
            if why != self.was:   # one call site per severity, as rclpy requires
                if why:
                    self.get_logger().warning(why)
                else:
                    self.get_logger().info('ok')
                self.was = why
            if self.logic.tick(now):
                self.get_logger().error(
                    f"autonomy while {why}: requesting hold {self.cfg['cause']}",
                    throttle_duration_sec=5.0)
                self.request.publish(String(data=json.dumps({
                    'op': 'hold', 'cause': self.cfg['cause'], 'by': name,
                    'token': self.token})))

    return HoldWatchdog


def run(make, args=None):
    """Spin the node class ``make()`` returns until shutdown."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = make()()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
