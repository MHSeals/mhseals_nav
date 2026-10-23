"""Request MAVLink message rates from MAVROS, again after every FCU reconnect.

MAVROS 2 reopens the FCU endpoint after a reboot, but the FCU forgets the
SET_MESSAGE_INTERVAL requests, so they are sent on each connected false->true edge.
"""
import threading
import time


def parse_rates(entries):
    """['33:20.0', ...] -> [(33, 20.0), ...]."""
    return [(int(i), float(hz)) for i, hz in (entry.split(':') for entry in entries)]


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


def main(args=None):
    import rclpy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import MessageInterval
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node('mavros_setup')
    rates = parse_rates(node.declare_parameter('message_rates', ['33:20.0']).value)
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
                request_rates(rates, call, node.get_logger().info)

    node.create_subscription(State, '/mavros/state', receive_state, 10)
    threading.Thread(target=work, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
