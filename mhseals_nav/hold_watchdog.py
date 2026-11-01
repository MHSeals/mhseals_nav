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
once; the mux forwards auto (ramped in from zero over its ramp) for the round trip until the
hold lands, its state and then the request on DDS, longer for a watchdog on the Orin. A mux
state older than mux_state_max_age_s, none, an unreadable one, or one while /command_mux/state
has other than one publisher (a bag replay, a stray sim mux) counts every command, as before.
approach_guard is not filtered under rc or manual: it requests its hold at confirmation, as
before. Its condition is transient, so one seen under rc, manual or a mux hold also leaves a
hold owed (``owed``) that never expires and goes out on the return to auto.
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
MUX_KEYS = ('mux_state_topic', 'mux_state_max_age_s')   # required in every watchdog's YAML


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
        self.owed = False        # a hold due under rc/manual, still to send (transient causes)

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
            source = json.loads(state)['input']
        except (ValueError, TypeError, KeyError):
            source = None
        self.mux_input = source if isinstance(source, str) else None
        self.mux_at = now

    def mux_unknown(self):
        """Forget the mux state (its topic has other than one publisher)."""
        self.mux_input = None

    def mux_known(self, now):
        limit = self.cfg.get('mux_state_max_age_s')   # the node refuses to start without it
        return self.mux_input is not None and limit is not None and fresh(
            self.mux_at, now, limit)

    def outranked(self, now):
        return self.mux_known(now) and self.mux_input in OUTRANK_AUTO

    def commanding(self, now):
        """An auto command within the window, whatever the mux does with it."""
        return fresh(self.command_at, now, self.cfg['command_window_s'])

    def autonomy(self, now):
        if self.outranked(now):
            return False
        # Idle: the mux has let every command it had expire, so only one since counts.
        at = self.open_at if self.mux_known(now) and self.mux_input == 'idle' else self.command_at
        return fresh(at, now, self.cfg['command_window_s'])

    def due(self, now):
        """Whether a hold is due; a transient cause also owes one seen while outranked."""
        return bool(self.reason(now)) and self.autonomy(now)

    def tick(self, now):
        """Return True when a hold request should go out now."""
        if not self.due(now):
            return False
        if self.sent_at is not None and now - self.sent_at < self.cfg['repeat_s']:
            return False
        self.sent_at = now
        self.owed = False
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
            missing = [key for key in MUX_KEYS if cfg.get(key) is None]
            if missing:
                raise ValueError(f"{filename}: missing {', '.join(missing)}")
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

        def check_mux(self):
            """One mux or none: two publishers (a replay, a stray mux) could mask a hold."""
            if self.count_publishers(self.cfg['mux_state_topic']) != 1:
                self.logic.mux_unknown()

        def on_command(self, _):
            self.check_mux()
            self.logic.command(time.monotonic())
            self.tick()

        def on_mux(self, msg):
            self.logic.mux(time.monotonic(), msg.data)
            self.tick()   # back to auto with a command in the window: hold now

        def tick(self):
            self.check_mux()
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
                    f"autonomy while {why or 'a hold owed from under rc or manual'}: "
                    f"requesting hold {self.cfg['cause']}",
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
