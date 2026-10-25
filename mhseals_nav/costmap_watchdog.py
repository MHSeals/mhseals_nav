"""Hold the boat through command_mux while autonomy runs on a stale perception costmap.

B's Nav2 StaticLayer has no timeout: without this, a dead Orin leaves Nav2 driving on
the last obstacles it saw. The costmap alone is not enough: with the tracker dead, the
Orin's StaticLayer keeps the last track grid while /perception/costmap stays fresh, so
recent /perception/tracks are required too. Hold-only; the operator releases like any
sticky hold.
"""

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run


class Watchdog(HoldGate):
    """Fresh costmap and tracks; times are monotonic seconds."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.fresh_at = None
        self.tracks_at = None

    def stamp_ok(self, stamp_age):
        # Both ways: a stamp from the future (Orin clock ahead) must not pass as fresh.
        return abs(stamp_age) <= self.cfg['max_stamp_age_s']

    def costmap(self, now, stamp_age):
        if self.stamp_ok(stamp_age):
            self.fresh_at = now

    def tracks(self, now, stamp_age):
        if self.stamp_ok(stamp_age):
            self.tracks_at = now

    def stale_inputs(self, now):
        """Names of the inputs that are missing or older than their limit."""
        cfg = self.cfg
        return [name for name, at, limit in (
            ('costmap', self.fresh_at, cfg['max_age_s']),
            ('tracks', self.tracks_at, cfg['max_tracks_age_s']))
            if not fresh(at, now, limit)]

    def reason(self, now):
        stale = self.stale_inputs(now)
        return f"perception stale or missing: {', '.join(stale)}" if stale else ''


def node_class():
    import time
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time

    from mhseals_nav.msg import TrackArray

    def subscribe(node):
        cfg, logic, clock = node.cfg, node.logic, node.get_clock()

        def age(msg):
            stamp = Time.from_msg(msg.header.stamp, clock_type=clock.clock_type)
            return (clock.now() - stamp).nanoseconds * 1e-9

        def on_costmap(msg):
            logic.costmap(time.monotonic(), age(msg))

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, cfg['costmap_topic'], on_costmap, latched)
        node.create_subscription(OccupancyGridUpdate, cfg['updates_topic'], on_costmap, 10)
        node.create_subscription(
            TrackArray, cfg['tracks_topic'],
            lambda msg: logic.tracks(time.monotonic(), age(msg)),
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    return watchdog_class('costmap_watchdog', 'costmap_watchdog.yaml', Watchdog, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
