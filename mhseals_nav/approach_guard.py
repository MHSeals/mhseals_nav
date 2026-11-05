"""Hold the boat through command_mux while an obstacle closes in on the hull.

MPPI and the planner score only the path ahead, so a boat closing from behind or the side
was driven into at full speed although the lidar had it lethal in /perception/costmap from
7 m away. A static polygon around the hull (Nav2's collision_monitor) cannot tell that boat
from a gate buoy just passed: both sit 0.6 to 1 m off the quarter. Motion can: the costmap
is in a world frame (odom), where a buoy's lethal cells stay put however the boat moves and
a moving hull's leading edge keeps marking cells that were free a moment ago.

So: lethal cells within ``trigger`` of the hull rectangle that lie more than ``move``
from every known lethal cell (first lethal ``min_age`` ago or more in more than one
snapshot, last within ``memory``) are new, if they lie in a connected cluster of
``min_cluster`` cells (``cluster`` links): a hull is one, spray and wake are specks.
New cells are an approach once they form a chain
for ``confirm`` (``follow``): each step within ``max_speed`` of the last, so scattered
clutter (spray, wake) cannot chain, and up to ``miss`` without a new cell (lidar
dropouts) keeps a chain. Then, while autonomy
commands flow (a station-keeping zero counts), it asks command_mux for a hold like the
other hold-only watchdogs (hold_watchdog), and like them never releases: a hold ends the
task (run_task), and the operator resumes. With ``enforce`` false it only logs.

Only motion faster than about move / min_age in the odom frame is new: a boat closing
slower than that is never held for. Overtaken, the boat stops rather than keeping course
and speed as a stand-on vessel would: the threats here are robotic or unmanned and the
boat is slow, and stopping keeps it from turning across what only its lidar sees without
giving anything new the authority to move it.
"""

from mhseals_nav.hold_watchdog import HoldGate, node_class as watchdog_class, run
from mhseals_nav.nearest_obstacle import Grid, grid_pose, hull_gap, lethal_points
from mhseals_nav.yaml_params import hull
import numpy


def clusters(points, link, size):
    """Mask of the points in a connected cluster (links of at most ``link`` m) of ``size``+."""
    n = len(points)
    keep = numpy.zeros(n, bool)
    if n < size:
        return keep
    near = numpy.hypot(points[:, None, 0] - points[None, :, 0],
                       points[:, None, 1] - points[None, :, 1]) <= link
    seen = numpy.zeros(n, bool)
    for first in range(n):
        if seen[first]:
            continue
        member, frontier = numpy.zeros(n, bool), [first]
        member[first] = seen[first] = True
        while frontier:
            fresh = near[frontier].any(axis=0) & ~member
            member |= fresh
            frontier = list(numpy.nonzero(fresh)[0])
        seen |= member
        if member.sum() >= size:
            keep |= member
    return keep


class Guard(HoldGate):
    """Approach detection; times are monotonic seconds."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.half = (cfg['half_length'], cfg['half_width'])
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
        # Known: lethal at least min_age ago, in more than one snapshot, and seen again
        # within memory. A buoy the lidar loses near the hull and finds again (seconds
        # apart) is still known; a speck of spray seen once is not, so it cannot mask
        # the front of a hull passing over that spot later.
        for key in [k for k, (_, last, _) in self.cells.items() if now - last > cfg['memory']]:
            del self.cells[key]
        known = numpy.array([xy for first, last, xy in self.cells.values()
                             if now - first >= cfg['min_age'] and last > first]
                            ).reshape(-1, 2)
        for x, y in points:
            key = (round(x * 100), round(y * 100))
            if key in self.cells:
                self.cells[key][1] = now
            else:
                self.cells[key] = [now, now, (x, y)]
        fresh, fresh_gaps = numpy.zeros((0, 2)), numpy.zeros(0)
        if now - self.since >= cfg['warmup'] and len(points):
            gaps = hull_gap(points, pose, self.half)
            close = gaps <= cfg['trigger']
            near = points[close]
            # Only cells that can be within move of a near one (a dock has thousands).
            known = known[hull_gap(known, pose, self.half) <=
                          cfg['trigger'] + cfg['move'] + 0.01]
            if len(near):
                new = numpy.ones(len(near), bool)
                if len(known):
                    new = numpy.hypot(near[:, None, 0] - known[None, :, 0],
                                      near[:, None, 1] - known[None, :, 1]
                                      ).min(axis=1) > cfg['move']
                if new.any():
                    fresh, fresh_gaps = near[new], gaps[close][new]
                    solid = clusters(fresh, cfg['cluster'], cfg['min_cluster'])
                    fresh, fresh_gaps = fresh[solid], fresh_gaps[solid]
        self.follow(now, fresh, fresh_gaps)

    def follow(self, now, fresh, gaps):
        """Extend each chain to the new cells within its reach, start chains from the rest.

        A chain is [start, last time, x, y, gap, hits, snapshots]: it moves to the centroid
        of the new cells within ``max_speed`` times the time since its last extension (plus
        ``move``), so a speck beside a hull's many cells barely moves it. One that finds
        none survives ``miss``, so lidar dropouts keep it. It confirms after
        ``confirm`` when extended in ``min_hits`` of the snapshots it saw.
        """
        cfg, used, kept = self.cfg, numpy.zeros(len(fresh), bool), []
        for chain in self.chains:
            start, t, x, y, _, hits, seen = chain
            if len(fresh):
                d = numpy.hypot(fresh[:, 0] - x, fresh[:, 1] - y)
                reach = d <= cfg['max_speed'] * (now - t) + cfg['move']
                if reach.any():
                    used |= reach
                    cx, cy = fresh[reach].mean(axis=0)
                    kept.append([start, now, cx, cy, float(gaps[reach].min()), hits + 1,
                                 seen + 1])
                    continue
            if now - t < cfg['miss'] + 0.5 / cfg['rate']:   # timer jitter
                kept.append(chain[:6] + [seen + 1])
        for i in numpy.argsort(gaps):
            if not used[i]:
                kept.append([now, now, fresh[i][0], fresh[i][1], float(gaps[i]), 1, 1])
                used |= numpy.hypot(fresh[:, 0] - fresh[i][0],
                                    fresh[:, 1] - fresh[i][1]) <= cfg['move']
        # The oldest chains first; clutter starts many short ones, keep a bounded few.
        self.chains = sorted(kept, key=lambda c: c[0])[:cfg['max_chains']]
        live = [c for c in self.chains if c[1] == now and now - c[0] >= cfg['confirm']
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

    def due(self, now):
        """Hold at a confirmed approach under rc or manual too, and owe one for the return.

        Under rc or manual the hold goes out at confirmation, as before the mux filter.
        The approach is gone after recent, so one seen under rc, manual or a mux hold is
        also owed (it never expires) and goes out on the return to auto.
        """
        why = self.reason(now)
        if why and self.outranked(now) and self.commanding(now):
            self.owed = True
        if why and self.mux_input in ('rc', 'manual') and self.commanding(now):
            return True
        return bool(why or self.owed) and self.autonomy(now)

    def tick(self, now):
        """Return True when a hold request should go out now; never unless enforcing."""
        return super().tick(now) and self.cfg['enforce']

    def reason(self, now):
        recent = self.approach_at is not None and \
            now - self.approach_at <= self.cfg['recent']
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
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

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

        def observe():
            at = grid_pose(node, node.tf, grid, cfg['base_frame'], cfg['max_costmap_age'],
                           cfg['max_tf_age'])
            if at is None:
                guard.blind()   # stale inputs are costmap_watchdog's to hold on
                return
            guard.observe(time.monotonic(), lethal_points(
                grid.cells, grid.resolution, grid.origin, at[:2], cfg['crop'],
                cfg['lethal']), at)

        node.create_timer(1.0 / cfg['rate'], observe)

        # What it holds, or with enforce false would hold, on: every repeat at most.
        report = node.create_publisher(String, cfg['report_topic'], 10)

        def publish_report():
            if guard.reason(time.monotonic()) and guard.autonomy(time.monotonic()):
                report.publish(String(data=json.dumps({
                    'gap_m': round(guard.gap, 2), 'at': guard.at, 'enforce': cfg['enforce']})))

        node.create_timer(cfg['repeat'], publish_report)

    return watchdog_class('approach_guard', Guard, subscribe, hull())   # config/hull.yaml


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
