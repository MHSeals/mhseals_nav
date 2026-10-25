"""Hold the boat through command_mux when the operator's heartbeat goes stale.

The FCU's GCS failsafe is off and nothing on the boat drives through it, so without this a
lost operator link leaves Nav2 and the tasks driving. The operator host publishes its name
on heartbeat_topic at about 2 Hz. Enforced once a heartbeat has arrived this session, or
from the start when require_operator is set; never with no operator ever seen otherwise.
"""

from mhseals_nav.hold_watchdog import HoldGate, node_class as watchdog_class, run


class LinkWatchdog(HoldGate):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.beat_at = None

    def heartbeat(self, now):
        self.beat_at = now

    def reason(self, now):
        if self.beat_at is None:
            return 'no operator heartbeat yet' if self.cfg['require_operator'] else ''
        if now - self.beat_at > self.cfg['max_age_s']:
            return f"operator heartbeat older than {self.cfg['max_age_s']} s"
        return ''


def node_class():
    import time
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    def subscribe(node):
        def on_beat(msg):
            if node.logic.beat_at is None:
                node.get_logger().info(f'operator heartbeat from {msg.data or "?"}')
            node.logic.heartbeat(time.monotonic())
        node.create_subscription(String, node.cfg['heartbeat_topic'], on_beat,
                                 qos_profile_sensor_data)

    return watchdog_class('link_watchdog', 'link_watchdog.yaml', LinkWatchdog, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
