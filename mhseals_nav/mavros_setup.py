"""Request MAVLink message rates from MAVROS, again after every FCU reconnect.

MAVROS 2 reopens the FCU endpoint after a reboot, but the FCU forgets the
SET_MESSAGE_INTERVAL requests, so they are sent on each connected false->true edge.
A reboot shorter than MAVROS's connection timeout never drops that flag, so a backwards
step in SYSTEM_TIME.time_boot_ms also counts as a reconnect.

It also decodes EKF_STATUS_REPORT, which MAVROS has no plugin for, from the raw
MAVLink stream onto /fcu/ekf_status for the health checks.

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
    """LiveParams rule: the startup contract, so a live set cannot be looser than a launch.

    A dropped id is not reset implicitly; removal is an explicit id:0 (FCU default) or
    id:-1 (disable) entry. An empty list, duplicate ids and rates below -1 are refused.
    """
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
    # A changed list is requested at once; the FCU keeps the latest request.
    live = LiveParams(node, {'message_rates': rates_rule},
                      on_change=lambda values, changed: wanted.set())
    client = node.create_client(MessageInterval, '/mavros/set_message_interval')
    wanted = threading.Event()
    # Not set at start: the first connected /mavros/state is a False->True edge and asks.
    connected = [False]
    last_reboot = [None]

    def reboot_seen():
        # One reboot shows as both a state edge and a boot-clock reset; ask once.
        now = time.monotonic()
        if not is_duplicate_trigger(last_reboot[0], now, policy['reboot_dedupe_window']):
            wanted.set()
            last_reboot[0] = now

    def receive_state(state):
        if reconnected(connected[0], state.connected):
            reboot_seen()
        connected[0] = state.connected

    def call(message_id, hz):
        done = threading.Event()
        request = MessageInterval.Request(message_id=message_id, message_rate=hz)
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return done.wait(policy['response_timeout']) and future.result().success

    def work():
        while rclpy.ok():
            wanted.wait()
            wanted.clear()
            if client.wait_for_service(timeout_sec=policy['service_wait']):
                request_rates(live['message_rates'], call, node.get_logger().info,
                              attempts=policy['retry_attempts'], wait=policy['retry_delay'])
            else:
                wanted.set()  # the wait already took 10 s; try again rather than skip

    ekf_pub = node.create_publisher(DiagnosticStatus, '/fcu/ekf_status', 10)

    boot_ms = [None]

    def receive_mavlink(msg):
        if not from_fcu(msg.sysid, msg.compid, policy['fcu_sysid']):
            return
        now_ms = decode_boot_ms(msg.msgid, msg.len, msg.payload64)
        # A routed GCS SYSTEM_TIME has its own boot clock and would look like a reboot.
        if now_ms is not None:
            if rebooted(boot_ms[0], now_ms):
                node.get_logger().info('FCU boot time went backwards; requesting rates again')
                reboot_seen()
            boot_ms[0] = now_ms
        status = decode_ekf_status(msg.msgid, msg.len, msg.payload64)
        if status is None:
            return
        level, message, values = ekf_status_values(status)
        ekf_pub.publish(DiagnosticStatus(
            level=bytes([level]), name='fcu_ekf', message=message,
            values=[KeyValue(key=k, value=v) for k, v in values.items()]))

    node.create_subscription(Mavlink, '/uas1/mavlink_source', receive_mavlink,
                             qos_profile_sensor_data)
    node.create_subscription(State, '/mavros/state', receive_state, 10)

    # Checked only while connected and after a full window, so start-up and reboots
    # do not warn before the rate request has had time to take effect.
    rc_arrivals = collections.deque()
    rc_low = [False]
    rc_since = [None]

    def check_rc_rate():
        now = time.monotonic()
        if not connected[0]:
            rc_since[0] = None
            return
        rc_since[0] = rc_since[0] or now
        if now - rc_since[0] < policy['rc_grace']:
            return
        low = rc_rate_low(rc_arrivals, now, window=policy['rc_window'],
                          min_hz=policy['rc_min_hz'], min_channels=policy['rc_min_channels'])
        if low != rc_low[0]:
            rc_low[0] = low
            (node.get_logger().warning if low else node.get_logger().info)(
                f"rc rate low: /mavros/rc/in under {policy['rc_min_hz']:g} Hz "
                f"or under {policy['rc_min_channels']} channels; "
                'MANUAL will be choppy and stick takeover impossible' if low else
                'rc rate ok')

    node.create_subscription(
        RCIn, '/mavros/rc/in',
        lambda msg: rc_arrivals.append((time.monotonic(), len(msg.channels))),
        qos_profile_sensor_data)
    node.create_timer(policy['rc_timer'], check_rc_rate)
    threading.Thread(target=work, daemon=True).start()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
