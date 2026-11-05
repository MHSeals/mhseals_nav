"""The shape shared by the hold-only safety watchdogs (costmap, link, fence, FCU, EKF inputs).

Each watches one condition. While it is bad and autonomy commands flow (an auto command
within command_window), the watchdog asks command_mux for a hold with its own cause and
token, and asks again every repeat, so a release while the condition is still bad is held
again. The operator clears holds (U clears them all); with release_after set, a watchdog
also releases its own hold (its token, nobody else's) once its condition has been clear that
long and the FCU reports disarmed on fcu_state_topic. Armed, or FCU state unknown or older
than fcu_state_max_age, it never releases itself. An auto command counts whatever its
value (a zero is station-keeping), except while the mux's own ~/state says an input that
outranks auto has it (rc, manual or a hold): then the command reaches nothing, so it is not
autonomy, and neither is a command received then once the mux has gone idle. Holds, the
TUI's neutral zeros on a hold or a lapsed lease and keyboard drive no longer cascade into
these holds (11-01: every mux hold brought link and fence holds 14-24 ms later). When the
mux reports auto again (a release, manual timing out), a command in the window counts at
once; the mux forwards auto (ramped in from zero over its ramp) for the round trip until the
hold lands, its state and then the request on DDS, longer for a watchdog on the Orin. A mux
state older than mux_state_max_age, none, an unreadable one, or one while /command_mux/state
has other than one publisher (a bag replay, a stray sim mux) counts every command, as before.
approach_guard is not filtered under rc or manual: it requests its hold at confirmation, as
before. Its condition is transient, so one seen under rc, manual or a mux hold also leaves a
hold owed (``owed``) that never expires and goes out on the return to auto.
Ages are receipt ages on the monotonic
clock; only costmap_watchdog also checks header stamps, so there a skew between the boat and
the Orin beyond max_stamp_age holds.

Each also publishes ``~/status`` (JSON ``{"ok", "reason"}``) every STATUS_S: its liveness for an
operator TUI, which refuses and holds arming within about 2 s of the beats stopping, long before
DDS would drop the dead node from the graph.
"""

import json
import os
from pathlib import Path
import secrets
import tempfile
import time

STATUS_S = 0.5    # s between ~/status beats; a TUI calls a watchdog dead after 2 s without one
# command_mux ~/state inputs that outrank auto: while one has the mux, /cmd_vel reaches nothing.
OUTRANK_AUTO = ('rc', 'manual', 'hold')
MUX_KEYS = ('mux_state_topic', 'mux_state_max_age')   # required in every watchdog's YAML


def fresh(at, now, limit):
    return at is not None and now - at <= limit


def mux_state(state):
    """Return a command_mux ~/state (its JSON) as (input, body); input None when unreadable."""
    try:
        body = json.loads(state)
        source = body['input']
    except (ValueError, TypeError, KeyError):
        body, source = {}, None
    return (source if isinstance(source, str) else None), body


def mux_defaults():
    """watchdogs.yaml's mux_state_topic and mux_state_max_age (its defaults), for other readers."""
    from mhseals_nav.navigation_config import load_strict
    from mhseals_nav.yaml_params import config_path, WATCHDOGS
    path = config_path(WATCHDOGS)
    defaults = load_strict(path.read_text(), path)['defaults']
    return {key: defaults[key] for key in MUX_KEYS}


def token(name):
    """Return the watchdog ``name``'s hold token, kept as long as command_mux keeps its holds.

    ``$XDG_RUNTIME_DIR/mhseals/watchdog_<name>_<ROS_DOMAIN_ID>.token`` (0600), beside
    hold_store's file: a restarted watchdog still releases its own hold, a reboot clears both.
    The first writer wins whole (linked in from a temp file), so two starts share one token.
    """
    base = os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir()
    domain = os.environ.get('ROS_DOMAIN_ID') or '0'
    path = Path(base) / 'mhseals' / f'watchdog_{name}_{domain}.token'
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or not path.read_text().strip():
        fd, temp = tempfile.mkstemp(dir=path.parent)   # 0600
        with os.fdopen(fd, 'w') as out:
            out.write(secrets.token_hex(8))
        try:
            os.link(temp, path)
        except FileExistsError:   # another start won, or an empty file: replace only that
            if not path.read_text().strip():
                os.replace(temp, path)
        finally:
            Path(temp).unlink(missing_ok=True)
    return path.read_text().strip()


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
        self.mux_causes = None   # the hold causes the mux reports, None when unread
        self.holding = False     # a hold of ours went out and has not been released since
        self.clear_at = None     # since when the condition has been clear while holding
        self.armed = None        # the FCU's armed flag, None when unknown
        self.armed_at = None

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
        self.mux_input, body = mux_state(state)
        self.mux_at = now
        try:
            self.mux_causes = {h['cause'] for h in body['holds']}
        except (KeyError, TypeError):
            self.mux_causes = None

    def fcu(self, now, armed):
        """Note the FCU's armed flag (its MAVROS state)."""
        self.armed, self.armed_at = bool(armed), now

    def disarmed(self, now):
        limit = self.cfg.get('fcu_state_max_age')
        return self.armed is False and limit is not None and fresh(self.armed_at, now, limit)

    def release(self, now):
        """Return True when our own hold should be released now (see release_after)."""
        after = self.cfg.get('release_after')
        if not self.holding or after is None:
            return False
        if self.reason(now) or self.owed:
            self.clear_at = None
            return False
        if self.clear_at is None:
            self.clear_at = now
        if now - self.clear_at < after or not self.disarmed(now):
            return False
        self.holding, self.clear_at = False, None
        # Checked only now, well after the last hold went out, so the state has caught up.
        cleared = self.mux_known(now) and self.mux_causes is not None \
            and self.cfg['cause'] not in self.mux_causes
        return not cleared                # already cleared (U): nothing to send

    def mux_unknown(self):
        """Forget the mux state (its topic has other than one publisher)."""
        self.mux_input = None

    def mux_known(self, now):
        limit = self.cfg.get('mux_state_max_age')   # the node refuses to start without it
        return self.mux_input is not None and limit is not None and fresh(
            self.mux_at, now, limit)

    def outranked(self, now):
        return self.mux_known(now) and self.mux_input in OUTRANK_AUTO

    def commanding(self, now):
        """Return whether an auto command arrived within the window, whatever the mux does."""
        return fresh(self.command_at, now, self.cfg['command_window'])

    def autonomy(self, now):
        if self.outranked(now):
            return False
        # Idle: the mux has let every command it had expire, so only one since counts.
        at = self.open_at if self.mux_known(now) and self.mux_input == 'idle' else self.command_at
        return fresh(at, now, self.cfg['command_window'])

    def due(self, now):
        """Whether a hold is due; a transient cause also owes one seen while outranked."""
        return bool(self.reason(now)) and self.autonomy(now)

    def tick(self, now):
        """Return True when a hold request should go out now."""
        if not self.due(now):
            return False
        if self.sent_at is not None and now - self.sent_at < self.cfg['repeat']:
            return False
        self.sent_at = now
        self.owed = False
        self.holding, self.clear_at = True, None
        return True


def node_class(name, logic, subscribe, base=None):
    """Build a watchdog node: ``logic(cfg)`` decides, ``subscribe(node)`` feeds ``node.logic``.

    ``cfg`` is ``base`` < the node's section of config/watchdogs.yaml < its parameters.

    Built lazily so the decision logic imports without rclpy. Keyword arguments of the
    returned class go to rclpy's Node (tests pass parameter_overrides).
    """
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    from mhseals_nav.yaml_params import node_watchdog_config

    class HoldWatchdog(Node):
        def __init__(self, **kwargs):
            super().__init__(name, automatically_declare_parameters_from_overrides=True,
                             **kwargs)
            self.cfg = cfg = node_watchdog_config(self, name, base)
            missing = [key for key in MUX_KEYS if cfg.get(key) is None]
            if missing:
                raise ValueError(f"watchdogs.yaml {name}: missing {', '.join(missing)}")
            self.logic = logic(cfg)
            self.token = token(name)
            self.request = self.create_publisher(String, cfg['request_topic'], 10)
            self.create_subscription(Twist, cfg['command_topic'], self.on_command, 10)
            # Latched like the mux publishes it, so the state is known from the start.
            self.create_subscription(String, cfg['mux_state_topic'], self.on_mux, QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            if cfg.get('release_after') is not None:
                from mavros_msgs.msg import State
                from rclpy.qos import qos_profile_sensor_data
                self.create_subscription(
                    State, cfg['fcu_state_topic'],
                    lambda m: self.logic.fcu(time.monotonic(), m.armed), qos_profile_sensor_data)
            subscribe(self)
            self.create_timer(1.0 / cfg['rate'], self.tick)
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
            elif self.logic.release(now):
                self.get_logger().info(
                    f"clear for {self.cfg['release_after']} s and the FCU disarmed: "
                    f"releasing my hold {self.cfg['cause']}")
                self.request.publish(String(data=json.dumps({
                    'op': 'release', 'cause': self.cfg['cause'], 'by': name,
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
