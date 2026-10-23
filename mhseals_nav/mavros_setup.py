"""Request MAVLink message rates from MAVROS, again after every FCU reconnect.

MAVROS 2 reopens the FCU endpoint after a reboot, but the FCU forgets the
SET_MESSAGE_INTERVAL requests, so they are sent on each connected false->true edge.
A reboot shorter than MAVROS's connection timeout never drops that flag, so a backwards
step in SYSTEM_TIME.time_boot_ms also counts as a reconnect.

It also decodes EKF_STATUS_REPORT, which MAVROS has no plugin for, from the raw
MAVLink stream onto /fcu/ekf_status for the health checks.
"""
import math
import struct
import threading
import time

from mhseals_nav.utils.live_params import LiveParams

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


def parse_rates(entries):
    """['33:20.0', ...] -> [(33, 20.0), ...]; ValueError for a bad entry."""
    rates = [(int(i), float(hz)) for i, hz in (entry.split(':') for entry in entries)]
    if any(i < 0 or not math.isfinite(hz) for i, hz in rates):
        raise ValueError('message ids must not be negative and rates must be finite')
    return rates  # MAVROS: rate < 0 disables a stream, 0 restores the FCU default


def with_removed(previous, rates):
    """rates plus rate 0 (FCU default) for ids in previous that rates dropped."""
    kept = {i for i, _ in rates}
    return rates + [(i, 0.) for i, _ in previous if i not in kept]


def reconnected(was_connected, is_connected):
    """True on the false->true edge, when the FCU has forgotten its intervals."""
    return is_connected and not was_connected


def rebooted(previous_ms, boot_ms):
    """True when the FCU's time since boot went backwards, i.e. it restarted."""
    return previous_ms is not None and boot_ms < previous_ms


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


def main(args=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from mavros_msgs.msg import Mavlink, State
    from mavros_msgs.srv import MessageInterval
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=args)
    node = Node('mavros_setup')
    node.declare_parameter('message_rates', ['33:20.0'])
    # A changed list is requested at once; the FCU keeps the latest request.
    live = LiveParams(node, {'message_rates': parse_rates},
                      on_change=lambda values, changed: wanted.set())
    client = node.create_client(MessageInterval, '/mavros/set_message_interval')
    wanted = threading.Event()
    # Not set at start: the first connected /mavros/state is a False->True edge and asks.
    connected = [False]
    sent = [[]]  # last requested list, so dropped ids can be reset

    def receive_state(state):
        if reconnected(connected[0], state.connected):
            wanted.set()
        connected[0] = state.connected

    def call(message_id, hz):
        done = threading.Event()
        request = MessageInterval.Request(message_id=message_id, message_rate=hz)
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return done.wait(5.) and future.result().success

    def work():
        while rclpy.ok():
            wanted.wait()
            wanted.clear()
            if client.wait_for_service(timeout_sec=10.):
                rates = live['message_rates']
                request_rates(with_removed(sent[0], rates), call, node.get_logger().info)
                sent[0] = rates
            else:
                wanted.set()  # the wait already took 10 s; try again rather than skip

    ekf_pub = node.create_publisher(DiagnosticStatus, '/fcu/ekf_status', 10)

    boot_ms = [None]

    def receive_mavlink(msg):
        now_ms = decode_boot_ms(msg.msgid, msg.len, msg.payload64)
        if now_ms is not None:
            if rebooted(boot_ms[0], now_ms):
                node.get_logger().info('FCU boot time went backwards; requesting rates again')
                wanted.set()
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
    threading.Thread(target=work, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
