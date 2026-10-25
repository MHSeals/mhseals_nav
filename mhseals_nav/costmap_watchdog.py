"""Hold the boat through command_mux while autonomy runs on a stale perception costmap.

B's Nav2 StaticLayer has no timeout: without this, a dead Orin leaves Nav2 driving on
the last obstacles it saw. Hold-only; the operator releases like any sticky hold.
"""

import json
import secrets
import time


class Watchdog:
    """Decide when to (re)send a hold request; times are monotonic seconds."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.fresh_at = None
        self.command_at = None
        self.sent_at = None

    def costmap(self, now, stamp_age):
        if stamp_age <= self.cfg['max_stamp_age_s']:
            self.fresh_at = now

    def command(self, now):
        self.command_at = now

    def stale(self, now):
        return self.fresh_at is None or now - self.fresh_at > self.cfg['max_age_s']

    def autonomy(self, now):
        return self.command_at is not None and \
            now - self.command_at <= self.cfg['command_window_s']

    def tick(self, now):
        """Return True when a hold request should go out now."""
        if not (self.stale(now) and self.autonomy(now)):
            return False
        if self.sent_at is not None and now - self.sent_at < self.cfg['repeat_s']:
            return False
        self.sent_at = now
        return True


def node_class():
    """Build the ROS node class lazily so the decision logic imports without rclpy."""
    from geometry_msgs.msg import Twist
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String

    from mhseals_nav.yaml_params import node_config

    class CostmapWatchdog(Node):
        def __init__(self):
            super().__init__('costmap_watchdog',
                             automatically_declare_parameters_from_overrides=True)
            self.cfg = cfg = node_config(self, 'costmap_watchdog.yaml')
            self.logic = Watchdog(cfg)
            self.token = secrets.token_hex(8)
            self.request = self.create_publisher(String, cfg['request_topic'], 10)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                OccupancyGrid, cfg['costmap_topic'], self.on_costmap, latched)
            self.create_subscription(
                OccupancyGridUpdate, cfg['updates_topic'], self.on_costmap, 10)
            self.create_subscription(Twist, cfg['command_topic'], self.on_command, 10)
            self.create_timer(1.0 / cfg['rate_hz'], self.tick)
            self.was_stale = None

        def on_costmap(self, msg):
            stamp = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            self.logic.costmap(time.monotonic(), age)

        def on_command(self, _):
            self.logic.command(time.monotonic())
            self.tick()

        def tick(self):
            now = time.monotonic()
            stale = self.logic.stale(now)
            if stale != self.was_stale:
                log = self.get_logger().warning if stale else self.get_logger().info
                log(f"{self.cfg['costmap_topic']} {'stale or missing' if stale else 'fresh'}")
                self.was_stale = stale
            if self.logic.tick(now):
                self.get_logger().error(
                    f"autonomy on a stale costmap: requesting hold {self.cfg['cause']}",
                    throttle_duration_sec=5.0)
                self.request.publish(String(data=json.dumps({
                    'op': 'hold', 'cause': self.cfg['cause'], 'by': 'costmap_watchdog',
                    'token': self.token})))

    return CostmapWatchdog


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = node_class()()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
