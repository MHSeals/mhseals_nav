"""ROS side of rtk_base: the MAVROS RTCM sink, the /rtk/status publisher and the node loop."""

import json
import time

STATUS_TOPIC = '/rtk/status'                    # latched std_msgs/String JSON (base.StatusTracker)
ROVER_RAW = '/mavros/gpsstatus/gps1/raw'        # mavros_msgs/GPSRAW: the rover's fix type (RTK float/fixed)
ROVER_FIX = '/mavros/global_position/raw/fix'   # the rover's position for NTRIP picking and the GGA uplink

# MAVROS fragments up to 4 x 180 B; a longer message is dropped on its side with an error.
MAX_FRAME = 720
# MAVROS fragments in 180 B pieces and sends no empty terminator, so the FCU cannot tell where a
# 360 or 540 B frame ends and drops it. 720 B is ended by the full fourth fragment.
FRAGMENT = 180
# A frame older than this when it reaches us is worth less than the next epoch.
MAX_AGE = 2.0


class MavrosSink:
    """Corrections sink: sink(frame, stamp) publishes the frame whole to MAVROS, once, no acks.

    Staleness is judged here from the frame's read stamp; pacing and loss are MAVROS's and the FCU's.
    """

    def __init__(self, node, namespace='/mavros', clock=time.monotonic):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from mavros_msgs.msg import RTCM
        self.node, self.clock, self.message = node, clock, RTCM
        self.topic = namespace.rstrip('/') + '/gps_rtk/send_rtcm'
        self.publisher = node.create_publisher(RTCM, self.topic, QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE))
        self.counts = dict(sent=0, stale=0, oversize=0, split=0)
        self.warned, self.count = None, 0
        self.bytes, self.last = 0, None     # published payload bytes; clock stamp of the last frame
        self.ids = {}                       # RTCM message id -> clock stamp it was last sent

    def __call__(self, frame, stamp):
        n = self.node.count_publishers(self.topic)
        if self.warned is None or n != self.count:
            self.check_owner(n)
        if len(frame) > MAX_FRAME:
            self.counts['oversize'] += 1
            self.node.get_logger().warning(f'RTCM frame of {len(frame)} B dropped; MAVROS accepts {MAX_FRAME}')
            return
        if self.clock() - stamp > MAX_AGE:
            self.counts['stale'] += 1
            return
        frame = bytes(frame)
        if FRAGMENT < len(frame) < MAX_FRAME and len(frame) % FRAGMENT == 0:
            # The FCU gets a byte stream, so a 1 B tail ends the last fragment short and in order
            self.counts['split'] += 1
            parts = [frame[:-1], frame[-1:]]
        else:
            parts = [frame]
        for part in parts:
            message = self.message()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.data = list(part)
            self.publisher.publish(message)
        self.counts['sent'] += 1
        self.bytes += len(frame)
        self.last = self.clock()
        self.ids[int.from_bytes(frame[3:5], 'big') >> 4] = self.last

    def recent(self, window=10.0):
        """RTCM message ids sent in the last ``window`` s, ascending."""
        now = self.clock()
        return sorted(i for i, stamp in self.ids.items() if now - stamp <= window)

    def check_owner(self, n):
        """Warn once per other live node publishing corrections; two senders interleave frames."""
        self.count, self.warned = n, self.warned or set()
        if n <= 1:
            return
        others = [(i.node_name, i.node_namespace) for i in self.node.get_publishers_info_by_topic(self.topic)]
        me = (self.node.get_name(), self.node.get_namespace())
        if me in others:
            others.remove(me)
        new = set(others) - self.warned
        self.warned |= new
        if new:
            names = ', '.join(sorted(name for name, _ in new))
            self.node.get_logger().warning(f'another RTCM publisher is active ({names}); stop it to avoid mixed corrections')


def status_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def serve(config, survey=False, position=None, source='auto', pin=None):
    """Own the node: local base and/or NTRIP -> Corrections -> MavrosSink (+ Radio); status at 1 Hz.

    ``source``: auto (both; config priority picks), local (the USB base only) or ntrip (NTRIP only).
    """
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from mhseals_nav.rtk import ntrip
    from mhseals_nav.rtk.base import BaseSession, Radio, StatusTracker
    from mhseals_nav.rtk.cli import Printer, base_line
    from mhseals_nav.rtk.corrections import Corrections
    rclpy.init()
    node = rclpy.create_node('rtk_base')
    sink = MavrosSink(node)
    sinks = [sink] + ([Radio(config['radio'], config['radio_baud'])] if config['radio'] else [])
    settings = ntrip.load_settings(config)
    if pin:
        settings['pin'] = pin
    priority = {'local': 'base', 'ntrip': 'ntrip'}.get(source, config['priority'])
    mux = Corrections(sinks, priority, settings['near_km'], config=config)
    rough = ntrip.Position(settings['position_cache'], settings['fallback'])
    base = BaseSession([mux.gate('base')], config) if config['device'] and source != 'ntrip' else None
    net = None
    if source != 'local' and settings['enabled'] and settings['casters']:
        try:
            net = ntrip.NtripSource(settings, mux.gate('ntrip'), rough)
        except PermissionError as error:
            node.get_logger().warning(f'NTRIP disabled: {error}')
    if base is None and net is None:
        node.get_logger().error(f'source {source}: no base receiver and no NTRIP; nothing to send')
    mux.attach(base, net, rough)

    def on_fix(msg):
        if msg.status.status >= 0:      # STATUS_NO_FIX is -1
            rough.offer('rover', msg.latitude, msg.longitude, msg.altitude)

    node.create_subscription(NavSatFix, ROVER_FIX, on_fix, qos_profile_sensor_data)
    node.create_timer(1.0, mux.evaluate)
    tracker = StatusTracker(mux, sink, lambda: node.count_publishers(sink.topic),
                            lambda: node.count_subscribers(sink.topic))
    publisher = node.create_publisher(String, STATUS_TOPIC, status_qos())
    printer = Printer()

    def tick():
        status = tracker.status()
        publisher.publish(String(data=json.dumps(status)))
        line = printer.offer(base_line(status), time.monotonic())
        if line:
            print(time.strftime('%H:%M:%S') + '  ' + line, flush=True)

    node.create_timer(1.0, tick)
    if base:
        base.start(survey=survey, position=position, auto=config['auto_site'])
    if net:
        net.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for part in (base, net):
            if part:
                part.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def read_status(timeout=3.0):
    """(status dict or None, rover {fix_type, satellites_visible, h_acc} or None), waiting up to ``timeout`` s."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    got = {}
    rclpy.init()
    node = rclpy.create_node('rtk_status')
    try:
        node.create_subscription(String, STATUS_TOPIC, lambda m: got.__setitem__('status', json.loads(m.data)), status_qos())
        try:
            from mavros_msgs.msg import GPSRAW
            node.create_subscription(GPSRAW, ROVER_RAW, lambda m: got.__setitem__('rover', dict(
                fix_type=m.fix_type, satellites_visible=m.satellites_visible, h_acc=m.h_acc / 1000)),
                qos_profile_sensor_data)
        except ImportError:
            pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not ('status' in got and 'rover' in got):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return got.get('status'), got.get('rover')
