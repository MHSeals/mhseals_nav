"""Hold the boat through command_mux while an obstacle closes in on the hull.

MPPI and the planner score only the path ahead, so a boat closing from behind or the side
was driven into at full speed although the lidar had it lethal in /perception/costmap from
7 m away. A static polygon around the hull (Nav2's collision_monitor) cannot tell that boat
from a gate buoy just passed: both sit 0.6 to 1 m off the quarter. Motion can: the costmap
is in a world frame (odom), where a buoy's lethal cells stay put however the boat moves and
a moving hull's leading edge keeps marking cells that were free a moment ago.

So: lethal cells within ``trigger_m`` of the hull rectangle that lie more than ``move_m``
from every known lethal cell (first lethal ``min_age_s`` ago or more in more than one
snapshot, last within ``memory_s``) are new. They are an approach once they form a chain
for ``confirm_s`` (``follow``): each step within ``max_speed`` of the last, so scattered
clutter (spray, wake) cannot chain, and up to ``miss_s`` without a new cell (lidar
dropouts) keeps a chain. Then, while autonomy
commands flow (a station-keeping zero counts), it asks command_mux for a hold like the
other hold-only watchdogs (hold_watchdog), and like them never releases: a hold ends the
task (run_task), and the operator resumes. With ``enforce`` false it only logs.

Only motion faster than about move_m / min_age_s in the odom frame is new: a boat closing
slower than that is never held for. Overtaken, the boat stops rather than keeping course
and speed as a stand-on vessel would: the threats here are robotic or unmanned and the
boat is slow, and stopping keeps it from turning across what only its lidar sees without
giving anything new the authority to move it.
"""

import math

from mhseals_nav.hold_watchdog import HoldGate, node_class as watchdog_class, run
import numpy


def lethal_points(cells, resolution, origin, centre, radius, lethal=100):
    """World (x, y) of the lethal cells within ``radius`` of ``centre``, as an (N, 2) array."""
    x, y = centre
    cols = slice(max(0, int((x - radius - origin[0]) / resolution)),
                 max(0, int(math.ceil((x + radius - origin[0]) / resolution))))
    rows = slice(max(0, int((y - radius - origin[1]) / resolution)),
                 max(0, int(math.ceil((y + radius - origin[1]) / resolution))))
    ys, xs = numpy.nonzero(cells[rows, cols] >= lethal)
    return numpy.column_stack((origin[0] + (xs + cols.start + 0.5) * resolution,
                               origin[1] + (ys + rows.start + 0.5) * resolution))


def hull_gap(points, pose, half):
    """Distance from the hull rectangle at ``pose`` (x, y, yaw) to each world point."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    bx, by = c * dx + s * dy, -s * dx + c * dy
    return numpy.hypot(bx - numpy.clip(bx, -half[0], half[0]),
                       by - numpy.clip(by, -half[1], half[1]))


class Guard(HoldGate):
    """Approach detection; times are monotonic seconds."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.half = (cfg['half_length_m'], cfg['half_width_m'])
        self.cells = {}               # cell key -> [first seen, last seen] world (x, y)
        self.since = None             # observing without a gap since
        self.chains = []              # [start, last, x, y, gap, hits, seen]: new cells moving on
        self.approach_at = None       # last snapshot that confirmed an approach
        self.gap = None               # the approaching cell nearest the hull: its gap
        self.at = None                # its world (odom) x, y

    def observe(self, now, points, pose):
        """Feed one costmap snapshot (world lethal points near the boat) and the hull pose."""
        cfg = self.cfg
        self.since = now if self.since is None else self.since
        # Known: lethal at least min_age_s ago, in more than one snapshot, and seen again
        # within memory_s. A buoy the lidar loses near the hull and finds again (seconds
        # apart) is still known; a speck of spray seen once is not, so it cannot mask
        # the front of a hull passing over that spot later.
        for key in [k for k, (_, last, _) in self.cells.items() if now - last > cfg['memory_s']]:
            del self.cells[key]
        known = numpy.array([xy for first, last, xy in self.cells.values()
                             if now - first >= cfg['min_age_s'] and last > first]
                            ).reshape(-1, 2)
        for x, y in points:
            key = (round(x * 100), round(y * 100))
            if key in self.cells:
                self.cells[key][1] = now
            else:
                self.cells[key] = [now, now, (x, y)]
        fresh, fresh_gaps = numpy.zeros((0, 2)), numpy.zeros(0)
        if now - self.since >= cfg['warmup_s'] and len(points):
            gaps = hull_gap(points, pose, self.half)
            close = gaps <= cfg['trigger_m']
            near = points[close]
            # Only cells that can be within move_m of a near one (a dock has thousands).
            known = known[hull_gap(known, pose, self.half) <=
                          cfg['trigger_m'] + cfg['move_m'] + 0.01]
            if len(near):
                new = numpy.ones(len(near), bool)
                if len(known):
                    new = numpy.hypot(near[:, None, 0] - known[None, :, 0],
                                      near[:, None, 1] - known[None, :, 1]
                                      ).min(axis=1) > cfg['move_m']
                if new.any():
                    fresh, fresh_gaps = near[new], gaps[close][new]
        self.follow(now, fresh, fresh_gaps)

    def follow(self, now, fresh, gaps):
        """Extend each chain to the new cells within its reach, start chains from the rest.

        A chain is [start, last time, x, y, gap, hits, snapshots]: it moves to the centroid
        of the new cells within ``max_speed`` times the time since its last extension (plus
        ``move_m``), so a speck beside a hull's many cells barely moves it. One that finds
        none survives ``miss_s``, so lidar dropouts keep it. It confirms after
        ``confirm_s`` when extended in ``min_hits`` of the snapshots it saw.
        """
        cfg, used, kept = self.cfg, numpy.zeros(len(fresh), bool), []
        for chain in self.chains:
            start, t, x, y, _, hits, seen = chain
            if len(fresh):
                d = numpy.hypot(fresh[:, 0] - x, fresh[:, 1] - y)
                reach = d <= cfg['max_speed'] * (now - t) + cfg['move_m']
                if reach.any():
                    used |= reach
                    cx, cy = fresh[reach].mean(axis=0)
                    kept.append([start, now, cx, cy, float(gaps[reach].min()), hits + 1,
                                 seen + 1])
                    continue
            if now - t < cfg['miss_s'] + 0.5 / cfg['rate_hz']:   # timer jitter
                kept.append(chain[:6] + [seen + 1])
        for i in numpy.argsort(gaps):
            if not used[i]:
                kept.append([now, now, fresh[i][0], fresh[i][1], float(gaps[i]), 1, 1])
                used |= numpy.hypot(fresh[:, 0] - fresh[i][0],
                                    fresh[:, 1] - fresh[i][1]) <= cfg['move_m']
        # The oldest chains first; clutter starts many short ones, keep a bounded few.
        self.chains = sorted(kept, key=lambda c: c[0])[:cfg['max_chains']]
        live = [c for c in self.chains if c[1] == now and now - c[0] >= cfg['confirm_s']
                and c[5] >= cfg['min_hits'] * c[6]]
        if live:
            start, _, x, y, gap, _, _ = min(live, key=lambda c: c[4])
            self.approach_at, self.gap = now, gap
            self.at = [round(float(x), 2), round(float(y), 2)]

    def blind(self):
        """Forget what was seen: a gap in the costmap or TF must not read as motion."""
        self.cells.clear()
        self.since = None
        self.chains = []

    def tick(self, now):
        """Return True when a hold request should go out now; never unless enforcing."""
        return super().tick(now) and self.cfg['enforce']

    def reason(self, now):
        recent = self.approach_at is not None and \
            now - self.approach_at <= self.cfg['recent_s']
        return 'an obstacle is closing in on the hull' if recent else ''

    def describe(self, why):
        return f'{why}: {self.gap:.2f} m off' if self.gap is not None else why


def node_class():
    """Build the ROS node class lazily so the logic imports without rclpy."""
    import json
    import time
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from mhseals_nav.nearest_obstacle import Grid

    def subscribe(node):
        cfg, guard, grid = node.cfg, node.logic, Grid()
        node.tf = Buffer()
        node.tf_listener = TransformListener(node.tf, node)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, cfg['costmap_topic'],
                                 lambda m: grid.full(m, time.monotonic()), latched)
        node.create_subscription(OccupancyGridUpdate, cfg['updates_topic'],
                                 lambda m: grid.update(m, time.monotonic()), 10)

        def pose():
            if grid.cells is None or time.monotonic() - grid.received > cfg['max_costmap_age_s']:
                return None
            try:
                tf = node.tf.lookup_transform(grid.frame, cfg['base_frame'], Time())
            except TransformException:
                return None
            stamp = Time.from_msg(tf.header.stamp, clock_type=node.get_clock().clock_type)
            if (node.get_clock().now() - stamp).nanoseconds * 1e-9 > cfg['max_tf_age_s']:
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def observe():
            at = pose()
            if at is None:
                guard.blind()   # stale inputs are costmap_watchdog's to hold on
                return
            guard.observe(time.monotonic(), lethal_points(
                grid.cells, grid.resolution, grid.origin, at[:2], cfg['crop_m'],
                cfg['lethal']), at)

        node.create_timer(1.0 / cfg['rate_hz'], observe)

        # What it holds, or with enforce false would hold, on: every repeat_s at most.
        report = node.create_publisher(String, cfg['report_topic'], 10)

        def publish_report():
            if guard.reason(time.monotonic()) and guard.autonomy(time.monotonic()):
                report.publish(String(data=json.dumps({
                    'gap_m': round(guard.gap, 2), 'at': guard.at, 'enforce': cfg['enforce']})))

        node.create_timer(cfg['repeat_s'], publish_report)

    return watchdog_class('approach_guard', 'approach_guard.yaml', Guard, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
