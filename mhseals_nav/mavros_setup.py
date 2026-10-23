"""Request MAVLink message rates from MAVROS, again after every FCU reconnect.

MAVROS 2 reopens the FCU endpoint after a reboot, but the FCU forgets the
SET_MESSAGE_INTERVAL requests, so they are sent on each connected false->true edge.

It also decodes EKF_STATUS_REPORT, which MAVROS has no plugin for, from the raw
MAVLink stream onto /fcu/ekf_status for the health checks.
"""
import struct
import threading
import time

from mhseals_nav.utils.live_params import LiveParams

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
    if any(i < 0 or not hz >= 0 or hz == float('inf') for i, hz in rates):
        raise ValueError('message ids and rates must be finite and not negative')
    return rates


def reconnected(was_connected, is_connected):
    """True on the false->true edge, when the FCU has forgotten its intervals."""
    return is_connected and not was_connected


def request_rates(rates, call, log, attempts=3, wait=8., sleep=time.sleep):
    """Call (message id, Hz) until it succeeds; log one line per id."""
    for message_id, hz in rates:
        for attempt in range(attempts):
            if call(message_id, hz):
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
    wanted.set()  # once at start
    connected = [False]

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
                request_rates(live['message_rates'], call, node.get_logger().info)

    ekf_pub = node.create_publisher(DiagnosticStatus, '/fcu/ekf_status', 10)

    def receive_mavlink(msg):
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
