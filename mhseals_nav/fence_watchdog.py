"""Hold the boat through command_mux outside a geofence, or near its edge heading outward.

The FCU fence is off and could not stop the ODROID-driven thrusters anyway. The fence is a
polygon in lat/lon (the map origin moves with every start, navsat wait_for_datum is false)
or in map metres, and it lives only in fence_file (``fence.load_fence``), read again whenever
the file changes; no file, or an invalid one, is no fence. Position comes from the EKF
(/gps/filtered for lat/lon, /odom/global for map), heading and velocity from /odom/global.
No valid fence, or no fresh position or odometry, holds too.

It also publishes the fence it enforces on fence_topic (latched JSON, see ``state``): the raw
polygon in its own frame and the margin rules, so the TUI's goal gates check goals against this
one source (a lat/lon fence in lat/lon, through navsat's toLL, never a translated copy).

anchor_service (std_srvs/Trigger) re-anchors a map fence at the stopped boat (see ``anchor``);
a lat/lon course area stays where it is. marker_topic draws the fence for rviz, latched, in
marker_frame, marker_outside_color while the boat is outside.

The file's ``on_exit`` (alert | return) says what leaving does: alert holds outside; return,
under autonomy, first asks the procedure and task runners to pause (pause_topic, latched JSON,
see ``pause``) so none fights it, then after return_lead sends Nav2 one slow goal
(return_speed) to the nearest interior point (return_inset in). However the return ends, back
inside, not back within return_timeout, the operator (or any hold) taking the mux, or the goal
ending elsewhere, it holds the mux and stays paused until the operator or an agent releases the
geofence hold; still outside, it holds as alert does. status_topic carries ``status`` at
status_rate and on every change.
"""

import math

from mhseals_nav.fence import (closest, FenceFile, inside, moved, ON_EXIT, outline,  # noqa: F401
                               polygon, problem, project, reference)
from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run

NEAR_EDGE = 'within the margin heading outward'
OUTSIDE = 'outside the fence'
INSIDE = ('', NEAR_EDGE, 'on the fence')      # reasons with the boat in the fence


class FenceWatchdog(HoldGate):
    def __init__(self, cfg, fence=None, why='no fence loaded'):
        """``fence`` is ``load_fence``'s {'frame', 'points', 'on_exit'}, None (with ``why``) for
        none, which holds."""
        super().__init__(cfg)
        self.frame, self.points, self.on_exit, self.poly, self.ref, self.error = \
            None, None, 'alert', None, None, why
        self.p, self.p_at, self.raw = None, None, None
        self.fix_ll, self.fix_at = None, None
        self.map_p, self.map_at = None, None
        self.yaw, self.v, self.motion_at = 0.0, (0.0, 0.0), None
        self.edge_m = None
        self.returning = None    # {'at', 'goal', 'send_at', 'tries'} while a return is on
        self.return_failed = False   # until back inside: hold as alert does
        self.parked = None       # {'why', 'seen'} after a return, until its hold is released
        self.orders = []         # for the node: (go, (x, y, yaw)), (cancel, why), (failed, why)
        self.set_fence(fence, why)

    def position(self, now, a, b):
        """(lat, lon) for a lat/lon fence, else (x, y) in map."""
        if self.frame == 'latlon':
            self.fix_ll, self.fix_at = (a, b), now
        if self.poly is None:
            return
        self.p = project(self.points[:2], a, b) if self.frame == 'latlon' else (a, b)
        self.p_at, self.raw = now, (a, b)

    def fix(self, now, lat, lon):
        """The boat's lat/lon, kept whatever the frame, so a course area set live has it."""
        self.fix_ll, self.fix_at = (lat, lon), now
        if self.frame == 'latlon':
            self.position(now, lat, lon)

    def map_position(self, now, x, y):
        """The boat in map, which places a lat/lon fence's marker."""
        self.map_p, self.map_at = (x, y), now
        if self.frame == 'map':
            self.position(now, x, y)

    def set_fence(self, fence, why=''):
        """Enforce ``fence`` (see ``__init__``) from now on; None, with ``why``, is no fence.

        A return aimed at another polygon is called off, to be planned again on this one.
        """
        old = (self.frame, self.points)
        if fence is None:
            self.frame, self.points, self.poly, self.ref, self.error = None, None, None, None, why
        else:
            self.frame, self.points, self.on_exit = fence['frame'], fence['points'], \
                fence['on_exit']
            self.poly, self.error = polygon(self.frame, self.points), ''
            self.ref = reference(self.frame, self.points) if self.poly else None
        if (self.frame, self.points) == old:
            return
        self.p, self.p_at, self.raw = None, None, None
        if self.frame == 'latlon' and self.fix_ll is not None:   # placed by what it had
            self.position(self.fix_at, *self.fix_ll)
        elif self.frame == 'map' and self.map_p is not None:
            self.position(self.map_at, *self.map_p)
        if self.returning is not None:
            self.returning = None
            self.orders.append(('cancel', 'the fence changed'))

    def anchor(self, now):
        """Re-anchor the fence at the boat: (True, what) or (False, why not).

        A map fence's configured shape moves so its reference (``reference``) sits at the
        boat, as the start of the test placed it; margins, frame and shape stay as configured.
        A lat/lon course area is geographic: it stays where it is.
        """
        if self.poly is None:
            return False, 'no valid fence configured'
        if self.frame == 'latlon':
            return True, 'course area fixed'
        if not fresh(self.p_at, now, self.cfg['max_age']):
            return False, 'no boat position (missing or older than max_age)'
        if not fresh(self.motion_at, now, self.cfg['max_age']):
            return False, 'no odometry (missing or older than max_age)'
        if math.hypot(*self.v) > self.cfg['anchor_max_speed']:
            return False, f'boat moving ({math.hypot(*self.v):.1f} m/s)'
        self.points = moved(self.points, self.ref, self.raw)
        self.poly, self.ref = polygon(self.frame, self.points), self.raw
        self.position(self.p_at, *self.raw)       # the plane's origin moved with the fence
        return True, f'fence anchored at {self.raw[0]:.7g}, {self.raw[1]:.7g} ({self.frame})'

    def outline(self, now):
        """The marker's closed outline in map (see ``outline``), None while not placeable."""
        limit = self.cfg['max_age']
        live = fresh(self.p_at, now, limit) and fresh(self.map_at, now, limit)
        return outline(self.frame, self.points,
                       self.raw if live else None, self.map_p if live else None)

    def motion(self, now, yaw, vx, vy):
        """Heading and body velocity; the velocity is rotated into the fence plane (ENU)."""
        c, s = math.cos(yaw), math.sin(yaw)
        self.yaw, self.v, self.motion_at = yaw, (vx * c - vy * s, vx * s + vy * c), now

    def state(self, now, stamp_ns=0):
        """Return what fence_topic carries: the raw fence it enforces and its margin rules.

        Always the same keys. ``frame`` (None without a fence) is latlon ([lat, lon, ...]; a
        reader projects with ``project`` from the first vertex, as here) or map ([x, y, ...]);
        ``points`` is the enforced flat list when ``valid``, else None with ``error`` saying
        why. ``reason`` is the hold reason now ('' while all is well); ``stamp_ns`` the node's
        clock. A reader treats a message older than three periods (3 / fence_rate) as no fence.
        """
        cfg, valid = self.cfg, self.poly is not None
        return {'valid': valid, 'frame': self.frame,
                'points': list(self.points) if valid else None,
                'margin_m': cfg['margin'], 'outward_angle_deg': math.degrees(cfg['outward_angle']),
                'outward_speed_mps': cfg['outward_speed'], 'reason': self.reason(now),
                'error': '' if valid else self.error or 'no valid fence', 'stamp_ns': stamp_ns}

    def reason(self, now):
        """The hold reason: where the boat is, else why a finished return still holds."""
        return self.where(now) or (self.parked['why'] if self.parked else '')

    def where(self, now):
        """Why the boat's place breaks the fence ('' while all is well)."""
        cfg = self.cfg
        if self.poly is None:
            return 'no valid fence configured'
        if not fresh(self.p_at, now, cfg['max_age']):
            return 'position missing or stale'
        if not fresh(self.motion_at, now, cfg['max_age']):
            return 'odometry missing or stale'
        if not inside(self.p, self.poly):
            return OUTSIDE
        # Every edge within the margin counts, so a corner is covered from both sides.
        cos_limit = math.cos(cfg['outward_angle'])
        for c in closest(self.p, self.poly):
            dx, dy = c[0] - self.p[0], c[1] - self.p[1]
            d = math.hypot(dx, dy)
            if d >= cfg['margin']:
                continue
            if d < 1e-6:
                return 'on the fence'
            nx, ny = dx / d, dy / d
            if self.v[0] * nx + self.v[1] * ny > cfg['outward_speed'] or \
                    math.cos(self.yaw) * nx + math.sin(self.yaw) * ny > cos_limit:
                self.edge_m = d    # kept out of the reason, which is compared every tick
                return NEAR_EDGE
        return ''

    def describe(self, why):
        return f'{why} ({self.edge_m:.1f} m)' if why == NEAR_EDGE else why

    def status(self, now):
        """What status_topic carries. ``inside`` and ``distance_to_edge`` (m, to the nearest
        edge, either side) are None without a fence or a fresh position; ``corners`` is the
        flat list in ``frame`` (latlon: lat, lon; map: x, y), None without a valid fence."""
        valid = self.poly is not None
        live = valid and fresh(self.p_at, now, self.cfg['max_age'])
        edge = min(math.dist(c, self.p) for c in closest(self.p, self.poly)) if live else None
        return {'inside': inside(self.p, self.poly) if live else None,
                'distance_to_edge': round(edge, 2) if live else None, 'frame': self.frame,
                'on_exit': self.on_exit, 'corners': list(self.points) if valid else None,
                'returning': self.returning is not None, 'paused': self.pause()['paused']}

    def pause(self):
        """What pause_topic carries: paused from the start of a return until its hold is
        released, so the procedure and task runners yield to it and resume only then."""
        why = 'returning inside the fence' if self.returning else \
            self.parked['why'] if self.parked else ''
        return {'paused': bool(why), 'cause': self.cfg['cause'], 'by': 'fence_watchdog',
                'why': why}

    def mux(self, now, state):
        """As HoldGate's; a parked return ends once the mux showed its hold and dropped it."""
        super().mux(now, state)
        if self.parked is not None and self.mux_causes is not None:
            if self.cfg['cause'] in self.mux_causes:
                self.parked['seen'] = True
            elif self.parked['seen']:
                self.parked = None

    def park(self, why):
        if self.parked is None:
            self.parked = {'why': f'{why}: release the {self.cfg["cause"]} hold to resume',
                           'seen': False}

    def due(self, now):
        """A hold, except outside while a return is on (on_exit: return). A parked return
        asks for its hold whatever the commands until the mux shows it."""
        self.step(now)
        if self.returning is not None and self.where(now) == OUTSIDE:
            return False
        if self.parked is not None:
            return not self.parked['seen']
        return super().due(now)

    def step(self, now):
        """Advance the return (on_exit: return), leaving the node its orders."""
        why = self.where(now)
        if why in INSIDE:
            self.return_failed = False
        if self.returning is None:
            if why == OUTSIDE and self.on_exit == 'return' and self.parked is None \
                    and not self.return_failed and self.autonomy(now):
                goal = self.return_goal(now)
                if goal is None:
                    self.fail('no interior point to return to (or no map pose)')
                else:       # the pause goes out now, the goal once the runners yielded
                    self.returning = {'at': now, 'goal': goal, 'tries': 0,
                                      'send_at': now + self.cfg['return_lead']}
            return
        sending = self.returning['send_at']
        if sending is not None and now >= sending:
            self.returning.update(send_at=None, sent=now, tries=self.returning['tries'] + 1)
            self.orders.append(('go', self.returning['goal']))
        if self.on_exit != 'return':
            self.stop('on_exit is alert', failed=False)
        elif why not in INSIDE and why != OUTSIDE:
            self.stop(why, failed=True)
        elif self.outranked(now):
            self.stop(f'the mux input is {self.mux_input}', failed=True)
        elif why == OUTSIDE and now - self.returning['at'] > self.cfg['return_timeout']:
            self.stop('not back inside within return_timeout', failed=True)

    def stop(self, why, failed):
        self.returning = None
        self.orders.append(('cancel', why))
        if failed:
            self.fail(why)

    def fail(self, why):
        self.return_failed = True
        self.orders.append(('failed', why))
        self.park(f'fence return failed ({why})')

    def return_result(self, now, outcome):
        """The return goal ended (``outcome``: succeeded, aborted, rejected, ...): parked,
        and still outside holds as alert does. A goal refused while the runners' goals wind
        down (rejected, or ended otherwise than succeeded within return_lead of its send: a
        runner goal sent just before the pause replaced it) is sent again after return_lead,
        return_tries sends in all."""
        if self.returning is None:
            return
        refused = outcome.startswith('rejected') or (
            outcome != 'succeeded' and now - self.returning.get('sent', now) < self.cfg['return_lead'])
        if refused and self.returning['tries'] < self.cfg['return_tries']:
            self.returning['send_at'] = now + self.cfg['return_lead']
            return
        self.returning = None
        self.orders.append(('cancel', f'goal {outcome}'))   # nothing to cancel; lifts the cap
        why = self.where(now)
        if why in INSIDE:
            self.park(f'back inside the fence (return goal {outcome})')
        else:
            place = 'outside' if why == OUTSIDE else f'with {why}'
            self.fail(f'the return goal ended {place} ({outcome})')

    def return_goal(self, now):
        """(x, y, yaw) in marker_frame: return_inset inside the nearest edge point, facing
        it from the boat; None when there is no such point or no fresh map pose."""
        if self.frame == 'latlon' and not fresh(self.map_at, now, self.cfg['max_age']):
            return None
        p, poly, inset = self.p, self.poly, self.cfg['return_inset']
        c = min(closest(p, poly), key=lambda q: math.dist(q, p))
        mean = (sum(x for x, _ in poly) / len(poly), sum(y for _, y in poly) / len(poly))
        target = None
        for toward in ((c[0] - p[0], c[1] - p[1]), (mean[0] - c[0], mean[1] - c[1])):
            norm = math.hypot(*toward)
            if norm > 1e-6:
                t = (c[0] + inset * toward[0] / norm, c[1] + inset * toward[1] / norm)
                if inside(t, poly):
                    target = t
                    break
        if target is None:
            return None
        yaw = math.atan2(target[1] - p[1], target[0] - p[0])
        if self.frame == 'latlon':      # map is east/north through the boat, as ``outline``
            target = (self.map_p[0] + target[0] - p[0], self.map_p[1] + target[1] - p[1])
        return target[0], target[1], yaw


def node_class():
    import json
    import time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Point, PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import ColorRGBA, String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker

    def logic(cfg):
        return FenceWatchdog(cfg)      # no fence (a hold) until subscribe's first read

    def subscribe(node):
        cfg, dog = node.logic.cfg, node.logic
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        fence = node.create_publisher(String, cfg['fence_topic'], latched)
        marker = node.create_publisher(Marker, cfg['marker_topic'], latched)
        status = node.create_publisher(String, cfg['status_topic'], 10)
        pause = node.create_publisher(String, cfg['pause_topic'], latched)
        drawn, said = [], {'key': None, 'at': None, 'pause': None}

        def publish_marker(now):
            line, outside = dog.outline(now), dog.status(now)['inside'] is False
            if line is None or (drawn and drawn[-1] == (line, outside)):
                return
            msg = Marker(ns='fence', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
            msg.header.frame_id = cfg['marker_frame']      # stamp 0: rviz's latest TF
            msg.pose.orientation.w = 1.0
            msg.scale.x = float(cfg['marker_width'])
            rgba = cfg['marker_outside_color'] if outside else cfg['marker_color']
            msg.color = ColorRGBA(**dict(zip('rgba', map(float, rgba))))
            msg.points = [Point(x=x, y=y) for x, y in line]
            marker.publish(msg)
            drawn[:] = [(line, outside)]

        def report(force=False):
            """status_topic at status_rate and on a change (the distance aside)."""
            try:
                now = time.monotonic()
                body = dog.status(now)
                key = dict(body, distance_to_edge=None)
                if force or key != said['key'] or now - said['at'] >= 1.0 / cfg['status_rate']:
                    status.publish(String(data=json.dumps(body)))
                    if key['inside'] != (said['key'] or {}).get('inside'):
                        publish_marker(now)
                    said.update(key=key, at=now)
                asked = dog.pause()
                if asked != said['pause']:       # latched: the runners read it on change
                    pause.publish(String(data=json.dumps(asked)))
                    node.get_logger().warning(f"pause: {asked['why'] or 'lifted'}")
                    said['pause'] = asked
                drain()
            except Exception as error:      # noqa: BLE001 - logged, the hold timer goes on
                node.get_logger().error(f'fence status failed: {error}',
                                        throttle_duration_sec=10.0)
        node.create_timer(1.0 / cfg['rate'], report)

        # on_exit: return. One goal at a time; results of a goal given up are ignored.
        to_pose = ActionClient(node, NavigateToPose, cfg['return_action'])
        limit = node.create_publisher(SpeedLimit, cfg['speed_limit_topic'], 10)
        goal = {'n': 0, 'handle': None}

        def go(x, y, yaw):
            goal['n'] += 1
            n = goal['n']
            if not to_pose.server_is_ready():
                dog.return_result(time.monotonic(), f"rejected: no {cfg['return_action']} server")
                return
            limit.publish(SpeedLimit(percentage=False, speed_limit=float(cfg['return_speed'])))
            pose = PoseStamped()
            pose.header.frame_id = cfg['marker_frame']
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

            def accepted(future):
                handle = future.result()
                if n != goal['n']:
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                    return
                if handle is None or not handle.accepted:
                    dog.return_result(time.monotonic(), 'rejected')
                    return
                goal['handle'] = handle
                handle.get_result_async().add_done_callback(lambda r: ended(n, r))
            to_pose.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(accepted)

        def ended(n, future):
            if n != goal['n']:
                return
            goal['handle'] = None
            result = future.result()
            ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
            dog.return_result(time.monotonic(), 'succeeded' if ok else 'aborted or replaced')

        def cancel():
            goal['n'] += 1
            if goal['handle'] is not None:
                goal['handle'].cancel_goal_async()
                goal['handle'] = None
            limit.publish(SpeedLimit(percentage=False, speed_limit=0.0))   # lifts the cap

        def drain():
            orders, dog.orders = dog.orders, []
            for order, arg in orders:
                if order == 'go':
                    node.get_logger().warning(
                        f'outside the fence: returning to {arg[0]:.1f}, {arg[1]:.1f} '
                        f"({cfg['marker_frame']}) at {cfg['return_speed']} m/s")
                    go(*arg)
                elif order == 'cancel':
                    node.get_logger().info(f'fence return over: {arg}')
                    cancel()
                else:
                    node.get_logger().error(f'fence return failed: {arg}; holding (alert)')

        # The fence is fence_file alone, read again on every change (the TUI's course popup and
        # boat_agent fence write it whole): checked at fence_rate.
        watched = FenceFile(cfg['fence_file'])

        def reload():
            read = watched.poll()
            if read is None:
                return
            dog.set_fence(*read)
            if dog.poly is None:
                node.get_logger().error(f'no valid fence ({read[1]}): autonomy will be held')
            else:
                node.get_logger().warning(f'fence: {len(dog.poly)} vertices in {dog.frame}, '
                                          f'on_exit {dog.on_exit}, from {cfg["fence_file"]}')
            drawn.clear()
            report(force=True)

        def publish_fence():     # its own timer: a failure here never touches the holds
            try:
                reload()
                now = time.monotonic()
                state = dog.state(now, node.get_clock().now().nanoseconds)
                fence.publish(String(data=json.dumps(state)))
                publish_marker(now)
            except Exception as error:      # noqa: BLE001 - logged, the hold timer goes on
                node.get_logger().error(f'fence publish failed: {error}',
                                        throttle_duration_sec=10.0)
        node.create_timer(1.0 / cfg['fence_rate'], publish_fence)

        def on_anchor(_, response):
            response.success, response.message = dog.anchor(time.monotonic())
            if response.success:
                node.get_logger().info(response.message)
                drawn.clear()
                publish_fence()
            else:
                node.get_logger().warning(f'fence anchor refused: {response.message}')
            return response
        node.create_service(Trigger, cfg['anchor_service'], on_anchor)
        publish_fence()

        def on_odom(msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            now = time.monotonic()
            dog.motion(now, yaw, msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            dog.map_position(now, msg.pose.pose.position.x, msg.pose.pose.position.y)

        def on_fix(msg):
            if msg.status.status >= NavSatStatus.STATUS_FIX and \
                    math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                dog.fix(time.monotonic(), msg.latitude, msg.longitude)

        node.create_subscription(Odometry, cfg['odom_topic'], on_odom, qos_profile_sensor_data)
        # Whatever the frame: a lat/lon fence written live needs the fix at once.
        node.create_subscription(NavSatFix, cfg['gps_topic'], on_fix, qos_profile_sensor_data)

    return watchdog_class('fence_watchdog', logic, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
