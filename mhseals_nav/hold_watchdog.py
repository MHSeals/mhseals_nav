"""The shape shared by the hold-only safety watchdogs (costmap, link, fence, FCU, EKF inputs).

Each watches one condition. While it is bad and autonomy commands flow (an auto command
within command_window_s), the watchdog asks command_mux for a hold with its own cause and
token, and asks again every repeat_s, so a release while the condition is still bad is held
again. It never releases: the operator clears holds. An auto command counts whatever its
value (a zero is station-keeping), except while the mux's own ~/state says an input that
outranks auto has it (rc, manual or a hold): then the command reaches nothing, so it is not
autonomy, and neither is a command received then once the mux has gone idle. Holds, the
TUI's neutral zeros on a hold or a lapsed lease and keyboard drive no longer cascade into
these holds (11-01: every mux hold brought link and fence holds 14-24 ms later). When the
mux reports auto again (a release, manual timing out), a command in the window counts at
once. A mux state older than mux_state_max_age_s, or none, counts every command, as before.
Ages are receipt ages on the monotonic
clock; only costmap_watchdog also checks header stamps, so there a skew between the boat and
the Orin beyond max_stamp_age_s holds.

Each also publishes ``~/status`` (JSON ``{"ok", "reason"}``) every STATUS_S: its liveness for an
operator TUI, which refuses and holds arming within about 2 s of the beats stopping, long before
DDS would drop the dead node from the graph.
"""

import json
import secrets
import time

STATUS_S = 0.5    # s between ~/status beats; a TUI calls a watchdog dead after 2 s without one
# command_mux ~/state inputs that outrank auto: while one has the mux, /cmd_vel reaches nothing.
OUTRANK_AUTO = ('rc', 'manual', 'hold')


def fresh(at, now, limit):
    return at is not None and now - at <= limit


class HoldGate:
    """Decide when to (re)send a hold request; ``reason(now)`` is '' while all is well."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.command_at = None   # the newest auto command
        self.open_at = None      # the newest one received while no input outranked auto
        self.mux_input = None
        self.mux_at = None
        self.sent_at = None

    def reason(self, now):
        raise NotImplementedError

    def describe(self, why):
        """Return the reason as logged, with detail that changes too often to compare."""
        return why

    def command(self, now):
        self.command_at = now
        if not self.outranked(now):
            self.open_at = now

    def mux(self, now, state):
        """Note a command_mux ~/state (its JSON); one that can't be read is unknown."""
        try:
            self.mux_input = json.loads(state)['input']
        except (ValueError, TypeError, KeyError):
            self.mux_input = None
        self.mux_at = now

    def mux_known(self, now):
        return self.mux_input is not None and fresh(
            self.mux_at, now, self.cfg['mux_state_max_age_s'])

    def outranked(self, now):
        return self.mux_known(now) and self.mux_input in OUTRANK_AUTO

    def autonomy(self, now):
        if self.outranked(now):
            return False
        # Idle: the mux has let every command it had expire, so only one since counts.
        at = self.open_at if self.mux_known(now) and self.mux_input == 'idle' else self.command_at
        return fresh(at, now, self.cfg['command_window_s'])

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
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
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
            # Latched like the mux publishes it, so the state is known from the start.
            self.create_subscription(String, cfg['mux_state_topic'], self.on_mux, QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            subscribe(self)
            self.create_timer(1.0 / cfg['rate_hz'], self.tick)
            self.status = self.create_publisher(String, '~/status', 10)
            self.create_timer(STATUS_S, self.beat)
            self.was = None

        def beat(self):
            why = self.logic.reason(time.monotonic())
            self.status.publish(String(data=json.dumps({'ok': not why, 'reason': why})))

        def on_command(self, _):
            self.logic.command(time.monotonic())
            self.tick()

        def on_mux(self, msg):
            self.logic.mux(time.monotonic(), msg.data)
            self.tick()   # back to auto with a command in the window: hold now

        def tick(self):
            now = time.monotonic()
            why = self.logic.reason(now)
            if why != self.was:   # one call site per severity, as rclpy requires
                if why:
                    self.get_logger().warning(self.logic.describe(why))
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
