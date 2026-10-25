"""Hold the boat through command_mux when the operator's heartbeat goes stale.

The FCU's GCS failsafe is off and nothing on the boat drives through it, so without this a
lost operator link leaves Nav2 and the tasks driving. The operator host publishes its name
on heartbeat_topic at about 2 Hz. Only beats naming one of operator_hosts count, and never
one naming the boat itself: a TUI on the boat over SSH is not the operator. Enforced once a
heartbeat has arrived this session, or from the start when require_operator is set; never
with no operator ever seen otherwise.
"""

import socket

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run


def short(host):
    """'Tranquility.local' -> 'tranquility'."""
    return str(host).strip().lower().split('.')[0]


class LinkWatchdog(HoldGate):
    def __init__(self, cfg, own_host=None):
        super().__init__(cfg)
        self.beat_at = None
        own = short(socket.gethostname() if own_host is None else own_host)
        self.hosts = {short(h) for h in cfg['operator_hosts']} - {own, ''}

    def heartbeat(self, now, host):
        """Count a beat named after an operator host; return whether it counted."""
        if short(host) not in self.hosts:
            return False
        self.beat_at = now
        return True

    def reason(self, now):
        if self.beat_at is None:
            return 'no operator heartbeat yet' if self.cfg['require_operator'] else ''
        if not fresh(self.beat_at, now, self.cfg['max_age_s']):
            return f"operator heartbeat older than {self.cfg['max_age_s']} s"
        return ''


def node_class():
    import time
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    def subscribe(node):
        ignored = set()

        def on_beat(msg):
            first = node.logic.beat_at is None
            if node.logic.heartbeat(time.monotonic(), msg.data):
                if first:
                    node.get_logger().info(f'operator heartbeat from {msg.data}')
            elif msg.data not in ignored and len(ignored) < 16:  # bounded
                ignored.add(msg.data)
                node.get_logger().warning(
                    f'ignoring heartbeat from {msg.data!r}: not one of '
                    f'{sorted(node.logic.hosts)}')

        node.create_subscription(String, node.cfg['heartbeat_topic'], on_beat,
                                 qos_profile_sensor_data)

    return watchdog_class('link_watchdog', 'link_watchdog.yaml', LinkWatchdog, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
