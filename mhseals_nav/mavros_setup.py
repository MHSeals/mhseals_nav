"""Request MAVLink message rates from MAVROS, again after every FCU reconnect.

MAVROS 2 reopens the FCU endpoint after a reboot, but the FCU forgets the
SET_MESSAGE_INTERVAL requests, so they are sent on each connected false->true edge.
A reboot shorter than MAVROS's connection timeout never drops that flag, so a backwards
step in SYSTEM_TIME.time_boot_ms also counts as a reconnect.

It also decodes EKF_STATUS_REPORT, which MAVROS has no plugin for, from the raw
MAVLink stream onto /fcu/ekf_status for the health checks.

A request can fail (no ack while the link is busy at connect) or be accepted and not
take effect, which leaves the message at the FCU's SRx_ stream rate (ATTITUDE at 4 Hz on
2026-10-25). So the FCU's own messages are counted, and every ``verify_period`` the ids
arriving below 0.7 of their requested rate are requested again.

RC arbitration treats rc/in frames older than 0.3 s as lost, so at the FCU default of
2 Hz MANUAL is choppy and no stick takeover latches. It therefore watches the real
/mavros/rc/in rate and logs when it is low instead of leaving the mux to flap silently.
"""
import collections
import struct
import threading
import time

from mhseals_nav.mavros_configuration import (
    installed_setup_defaults, validate_rates, validate_setup)
from mhseals_nav.utils.live_params import LiveParams

MAV_COMP_ID_AUTOPILOT1 = 1
SYSTEM_TIME = 2
SYSTEM_TIME_BOOT = '<QI'  # time_unix_usec, time_boot_ms
EKF_STATUS_REPORT = 193
EKF_FIELDS = ('velocity_variance', 'pos_horiz_variance', 'pos_vert_variance',
              'compass_variance', 'terrain_alt_variance')
EKF_PAYLOAD = '<5fHf'  # five variances, flags, airspeed_variance (extension)

# EKF_STATUS_FLAGS bit -> name (ArduPilot; bits 11-14 are undefined).
EKF_FLAGS = {0: 'ATTITUDE', 1: 'VELOCITY_HORIZ', 2: 'VELOCITY_VERT', 3: 'POS_HORIZ_REL',
             4: 'POS_HORIZ_ABS', 5: 'POS_VERT_ABS', 6: 'POS_VERT_AGL', 7: 'CONST_POS_MODE',
             8: 'PRED_POS_HORIZ_REL', 9: 'PRED_POS_HORIZ_ABS', 10: 'UNINITIALIZED',
             15: 'GPS_GLITCHING'}


def rates_rule(value):
    """Validate live message_rates with the startup rule; no implicit 0 resets."""
    return validate_rates(list(value) if isinstance(value, (list, tuple)) else value)


def reconnected(was_connected, is_connected):
    """True on the false->true edge, when the FCU has forgotten its intervals."""
    return is_connected and not was_connected


def rebooted(previous_ms, boot_ms):
    """True when the FCU's time since boot went backwards, i.e. it restarted."""
    return previous_ms is not None and boot_ms < previous_ms


def rc_rate_low(arrivals, now, window=5., min_hz=10., min_channels=8):
    """True when rc/in has been too slow or too empty over the last window seconds.

    arrivals is a deque of (receive time, channel count); old entries are dropped.
    The 25 Hz request lands well above min_hz, the 2 Hz FCU default well below.
    """
    while arrivals and arrivals[0][0] < now - window:
        arrivals.popleft()
    good = sum(1 for _, channels in arrivals if channels >= min_channels)
    return good / window < min_hz


def rate_shortfalls(counts, span, wanted, tolerance=0.7):
    """(id, wanted Hz, measured Hz) for every requested id arriving below tolerance * wanted.

    counts maps message id to messages seen over span seconds, e.g. from a bag of
    /uas1/mavlink_source after connect or an FCU reboot. Ids wanted at 0 or less are skipped.
    """
    measured = {i: counts.get(i, 0) / span for i in wanted}
    return [(i, hz, measured[i]) for i, hz in sorted(wanted.items())
            if hz > 0 and measured[i] < tolerance * hz]


def from_fcu(sysid, compid, fcu_sysid=1):
    """True for the autopilot's own messages, not a routed GCS or companion's."""
    return sysid == fcu_sysid and compid == MAV_COMP_ID_AUTOPILOT1


def decode_boot_ms(msgid, length, payload64):
    """time_boot_ms of a SYSTEM_TIME message, or None for other messages."""
    if msgid != SYSTEM_TIME:
        return None
    raw = struct.pack(f'<{len(payload64)}Q', *payload64)[:length]
    raw = raw.ljust(struct.calcsize(SYSTEM_TIME_BOOT), b'\0')
    return struct.unpack(SYSTEM_TIME_BOOT, raw[:struct.calcsize(SYSTEM_TIME_BOOT)])[1]


def request_rates(rates, call, log, attempts=3, wait=8., sleep=time.sleep):
    """Call (message id, Hz) until it succeeds; log one line per id.

    A call that raises counts as a failed attempt, so the worker thread survives it.
    """
    for message_id, hz in rates:
        for attempt in range(attempts):
            try:
                ok = call(message_id, hz)
            except Exception as error:
                log(f'message {message_id} at {hz:g} Hz: {error!r}')
                ok = False
            if ok:
                log(f'message {message_id} at {hz:g} Hz: ok')
                break
            if attempt < attempts - 1:
                sleep(wait)
        else:
            log(f'message {message_id} at {hz:g} Hz: failed')


def is_duplicate_trigger(last, now, window=5.):
    """True when a reboot signal follows another within `window` s (state edge plus boot clock)."""
    return last is not None and now - last < window


def decode_ekf_status(msgid, length, payload64):
    """Raw MAVLink fields -> {flags, names, variances...}, or None for other messages.

    MAVLink 2 drops trailing zero bytes, so the payload is padded back to full size.
    """
    if msgid != EKF_STATUS_REPORT:
        return None
    raw = struct.pack(f'<{len(payload64)}Q', *payload64)[:length]
    raw = raw.ljust(struct.calcsize(EKF_PAYLOAD), b'\0')
    *variances, flags, airspeed = struct.unpack(EKF_PAYLOAD, raw)
    fields = dict(zip(EKF_FIELDS + ('airspeed_variance',), variances + [airspeed]))
    names = [name for bit, name in EKF_FLAGS.items() if flags >> bit & 1]
    return {'flags': flags, 'names': names, **fields}


def ekf_status_values(status):
    """Turn a decoded status into (level, message, values) for a DiagnosticStatus."""
    values = {key: f'{value:.4g}' for key, value in status.items() if key.endswith('variance')}
    values['flags'] = str(status['flags'])
    values['const_pos_mode'] = str('CONST_POS_MODE' in status['names']).lower()
    stuck = 'CONST_POS_MODE' in status['names'] or 'ATTITUDE' not in status['names']
    return (1 if stuck else 0), ','.join(status['names']), values


class RateKeeper:
    """When to request which message rates: reconnects, reboots and rates that fall short.

    want() wakes the worker for some ids or the whole list; take() hands them over.
    Not set at start: the first connected state is a false->true edge and asks.
    """

    def __init__(self, policy, clock=time.monotonic):
        self.policy, self.clock = policy, clock
        self.wanted = threading.Event()
        self.lock = threading.Lock()  # subset and wanted change together
        self.subset = None            # ids to request on the next wake; None is the whole list
        self.busy = False             # a request round is running: its arrivals are not counted
        self.connected = False
        self.last_reboot = None
        self.boot_ms = None
        self.counts = collections.Counter()  # FCU messages per id since span
        self.span = None              # start of the counting span; None while not counting

    def want(self, ids=None):
        """Wake the worker for ``ids``, or the whole list; a pending whole list wins."""
        with self.lock:
            if ids is None or (self.wanted.is_set() and self.subset is None):
                self.subset = None
            else:
                self.subset = set(ids) | (self.subset or set())
            self.wanted.set()

    def take(self):
        """Wait for a wake; the ids to request, None for the whole list."""
        self.wanted.wait()
        with self.lock:
            self.wanted.clear()
            ids, self.subset = self.subset, None
        return ids

    def reboot_seen(self):
        # One reboot shows as both a state edge and a boot-clock reset; ask once.
        now = self.clock()
        if not is_duplicate_trigger(self.last_reboot, now, self.policy['reboot_dedupe_window']):
            self.want()
            self.last_reboot = now

    def on_state(self, connected):
        if reconnected(self.connected, connected):
            self.reboot_seen()
        self.connected = connected

    def on_message(self, msgid, boot_ms=None):
        """Count an FCU message; True when its boot clock went backwards (a reboot)."""
        self.counts[msgid] += 1
        if boot_ms is None:
            return False
        went_back = rebooted(self.boot_ms, boot_ms)
        if went_back:
            self.reboot_seen()
        self.boot_ms = boot_ms
        return went_back

    def verify(self, wanted):
        """Request again the ids arriving below 0.7 of their rate over the last period.

        wanted maps id to Hz; returns the shortfalls requested again.
        """
        now = self.clock()
        if not self.connected or self.busy or self.wanted.is_set():
            self.span = None
            return []
        if self.span is None or now - self.span < self.policy['verify_period']:
            if self.span is None:
                self.counts.clear()
                self.span = now
            return []
        short = rate_shortfalls(dict(self.counts), now - self.span, wanted)
        self.counts.clear()
        self.span = now
        if short:
            self.want({i for i, _, _ in short})
        return short


class RcMonitor:
    """Whether /mavros/rc/in arrives fast and full enough, as low/ok transitions.

    Checked only while connected and after a grace period, so start-up and reboots
    do not warn before the rate request has had time to take effect.
    """

    def __init__(self, policy, clock=time.monotonic):
        self.policy, self.clock = policy, clock
        self.arrivals = collections.deque()
        self.low = False
        self.since = None

    def arrive(self, channels):
        self.arrivals.append((self.clock(), channels))

    def check(self, connected):
        """'low' or 'ok' when the state changes, else None."""
        now = self.clock()
        if not connected:
            self.since = None
            return None
        self.since = self.since or now
        if now - self.since < self.policy['rc_grace']:
            return None
        low = rc_rate_low(self.arrivals, now, window=self.policy['rc_window'],
                          min_hz=self.policy['rc_min_hz'],
                          min_channels=self.policy['rc_min_channels'])
        if low == self.low:
            return None
        self.low = low
        return 'low' if low else 'ok'


def spin_until_shutdown(node):
    """Spin until SIGINT; the wait-set RCLError of a context shut down mid-spin is a clean exit."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from mavros_msgs.msg import Mavlink, RCIn, State
    from mavros_msgs.srv import MessageInterval
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    defaults = installed_setup_defaults()
    rclpy.init(args=args)
    node = Node('mavros_setup')
    for name, value in defaults.items():
        node.declare_parameter(name, value)
    policy = validate_setup({name: node.get_parameter(name).value for name in defaults})
    keeper = RateKeeper(policy)
    rc = RcMonitor(policy)
    # A changed list is requested at once; the FCU keeps the latest request.
    live = LiveParams(node, {'message_rates': rates_rule},
                      on_change=lambda values, changed: keeper.want())
    client = node.create_client(MessageInterval, '/mavros/set_message_interval')
    log = node.get_logger()

    def call(message_id, hz):
        done = threading.Event()
        request = MessageInterval.Request(message_id=message_id, message_rate=hz)
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return done.wait(policy['response_timeout']) and future.result().success

    def work():
        while rclpy.ok():
            ids = keeper.take()
            rates = live['message_rates']
            if ids is not None:
                rates = [(i, hz) for i, hz in rates if i in ids]
            if client.wait_for_service(timeout_sec=policy['service_wait']):
                keeper.busy = True
                try:
                    request_rates(rates, call, log.info, attempts=policy['retry_attempts'],
                                  wait=policy['retry_delay'])
                finally:
                    keeper.busy = False
            else:
                keeper.want()  # the wait already took 10 s; try again rather than skip

    ekf_pub = node.create_publisher(DiagnosticStatus, '/fcu/ekf_status', 10)

    def verify_rates():
        short = keeper.verify(dict(live['message_rates']))
        if short:
            log.info('rates short, requesting again: ' + ', '.join(
                f'{i} at {got:.1f} of {hz:g} Hz' for i, hz, got in short))

    def receive_mavlink(msg):
        # A routed GCS SYSTEM_TIME has its own boot clock and would look like a reboot.
        if not from_fcu(msg.sysid, msg.compid, policy['fcu_sysid']):
            return
        if keeper.on_message(msg.msgid, decode_boot_ms(msg.msgid, msg.len, msg.payload64)):
            log.info('FCU boot time went backwards; requesting rates again')
        status = decode_ekf_status(msg.msgid, msg.len, msg.payload64)
        if status is None:
            return
        level, message, values = ekf_status_values(status)
        ekf_pub.publish(DiagnosticStatus(
            level=bytes([level]), name='fcu_ekf', message=message,
            values=[KeyValue(key=k, value=v) for k, v in values.items()]))

    def check_rc_rate():
        # One severity per call site: rclpy raises when a site changes severity.
        change = rc.check(keeper.connected)
        if change == 'low':
            log.warning(
                f"rc rate low: /mavros/rc/in under {policy['rc_min_hz']:g} Hz "
                f"or under {policy['rc_min_channels']} channels; "
                'MANUAL will be choppy and stick takeover impossible')
        elif change == 'ok':
            log.info('rc rate ok')

    node.create_subscription(Mavlink, '/uas1/mavlink_source', receive_mavlink,
                             qos_profile_sensor_data)
    node.create_subscription(State, '/mavros/state',
                             lambda state: keeper.on_state(state.connected), 10)
    node.create_subscription(RCIn, '/mavros/rc/in', lambda msg: rc.arrive(len(msg.channels)),
                             qos_profile_sensor_data)
    node.create_timer(policy['rc_timer'], check_rc_rate)
    node.create_timer(policy['verify_period'], verify_rates)
    threading.Thread(target=work, daemon=True).start()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
