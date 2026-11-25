"""trial: drive a course through Nav2 (NavigateThroughPoses) and score it.

  ros2 run mhseals_nav trial COURSE [--scale S] [--box LxW]
  ros2 run mhseals_nav trial home

A course is config/trials/COURSE.yaml: ``points`` ([x, y] in the start pose's frame, x ahead, y left;
metres, or fractions of ``length`` metres when the file has one) and ``timeout_s``. ``home`` is one
goal back to the start pose of today's last trial ($ASTRO_DATA/<date>/trial_start.yaml). The course is
published as markers on /trial/course for RViz. --box LxW cancels the goal and stops the trial when the
boat leaves an L (along the start heading) by W rectangle centred on the course. The trial reads the start pose from odometry and drives the course one
leg per NavigateThroughPoses goal: a closed course ends where it starts, and Nav2's goal checker
looks only at a path's last pose, so one goal for the whole loop "arrives" at once. A leg counts
as reached when Nav2 succeeds and the boat is within --tolerance of its pose; the trial is
``done`` only when every leg is, else ``incomplete k/n``. It prints a progress line every second,
then the metrics (time, mean/max cross-track error, final position error, yaw-rate sign changes,
max speed, and the command-vs-actual tracking error per axis). The summary is appended to $ASTRO_DATA/<date>/log.txt
(``HH:MM:SS  trial   ...``) and published on /events, with a short hash of controller_server's live
parameters so two trials can be told apart by tuning. Ctrl-C, SIGTERM or SIGHUP (an ssh drop)
cancels the running goal before the trial exits.
"""

import argparse
import datetime
import hashlib
import math
import os
from pathlib import Path
import signal
import sys
import time

from mhseals_nav.layers import config_path, load_strict

LOG_KIND = 'trial'
YAW_DEADBAND = 0.05     # rad/s: yaw rates inside this are not a sign (noise around zero)


# --- pure -------------------------------------------------------------------------------------

HOME = 'home'
START_FILE = 'trial_start.yaml'


def courses():
    return sorted([p.stem for p in config_path('trials').glob('*.yaml')] + [HOME])


def load_course(name, scale=1.0, path=None):
    """(points [(x, y)] in the start frame, scaled; timeout_s; box text or None) of config/trials/<name>.yaml."""
    path = Path(path or config_path('trials', name + '.yaml'))
    if not path.is_file():
        raise ValueError(f'no course {name!r}; have: {", ".join(courses()) or "none"}')
    data = load_strict(path.read_text(), path) or {}
    points = data.get('points')
    if (not isinstance(points, list) or not points
            or not all(isinstance(p, list) and len(p) == 2 for p in points)):
        raise ValueError(f'{path}: points must be a list of [x, y]')
    if not (scale > 0 and math.isfinite(scale)):
        raise ValueError('--scale must be a positive number')
    scale *= float(data.get('length', 1.0))
    return ([(float(x) * scale, float(y) * scale) for x, y in points],
            float(data.get('timeout_s', 300)) * max(scale, 1.0), data.get('box'))


def parse_box(text):
    """'9x4' -> (9.0, 4.0): length along the start heading, width across."""
    try:
        length, width = (float(v) for v in text.lower().split('x'))
    except ValueError:
        raise ValueError(f'--box {text!r}: expected LENGTHxWIDTH in metres, e.g. 9x4') from None
    if not (length > 0 and width > 0 and math.isfinite(length + width)):
        raise ValueError(f'--box {text!r}: sides must be positive')
    return length, width


def box_centre(points):
    """Centre (start frame) of the course's extent, the start (0, 0) included."""
    xs, ys = [0.0] + [p[0] for p in points], [0.0] + [p[1] for p in points]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2


def box_offsets(point, start, box, centre):
    """(x, y, over_x, over_y): point in the box's frame and how far past each side pair (negative: inside)."""
    dx, dy = point[0] - start[0], point[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    x, y = c * dx + s * dy - centre[0], -s * dx + c * dy - centre[1]
    return x, y, abs(x) - box[0] / 2, abs(y) - box[1] / 2


def box_margin(points, start, box, centre):
    """The closest the run came to the box's edge, m (negative: it left)."""
    return min(-max(box_offsets(p, start, box, centre)[2:]) for p in points)


def outside_box(point, start, box, centre):
    """None while point (odometry frame) is inside the box, else one line of why."""
    x, y, over_x, over_y = box_offsets(point, start, box, centre)
    if over_x <= 0 and over_y <= 0:
        return None
    side = ('ahead' if x > 0 else 'behind') if over_x >= over_y else ('left' if y > 0 else 'right')
    return f'left the {box[0]:g}x{box[1]:g} m box {max(over_x, over_y):.2f} m {side}'


def start_path(data=None, when=None):
    data = data or os.environ.get('ASTRO_DATA')
    if not data:
        return None
    return Path(data).expanduser() / f'{(when or datetime.date.today()):%Y-%m-%d}' / START_FILE


def save_start(frame, start, course, data=None):
    """Record the trial's start pose (frame, x, y, yaw) for ``trial home``; returns the path or None."""
    path = start_path(data)
    if path is None:
        return None
    import yaml
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({'frame': frame, 'x': start[0], 'y': start[1], 'yaw': start[2],
                                    'course': course}))
    return path


def load_start(data=None):
    """(frame, (x, y, yaw)) of today's last trial start."""
    path = start_path(data)
    if path is None or not path.is_file():
        raise ValueError(f'no start pose of a trial today ({path or "ASTRO_DATA unset"}); run a course first')
    d = load_strict(path.read_text(), path)
    return d['frame'], (float(d['x']), float(d['y']), float(d['yaw']))


def to_world(points, start):
    """Course points from the start frame to the odometry frame; start is (x, y, yaw).

    Returns [(x, y, yaw)]: each pose faces along the leg that reaches it."""
    x0, y0, yaw0 = start
    c, s = math.cos(yaw0), math.sin(yaw0)
    out, prev = [], (x0, y0)
    for px, py in points:
        x, y = x0 + c * px - s * py, y0 + s * px + c * py
        heading = math.atan2(y - prev[1], x - prev[0]) if (x, y) != prev else (out[-1][2] if out else yaw0)
        out.append((x, y, heading))
        prev = (x, y)
    return out


def segment_distance(point, a, b):
    """Distance from point to segment ab."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if not length2 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - (a[0] + t * dx), point[1] - (a[1] + t * dy))


def cross_track(point, route):
    """Distance from point to the nearest segment of the route polyline."""
    if len(route) == 1:
        return math.hypot(point[0] - route[0][0], point[1] - route[0][1])
    return min(segment_distance(point, a, b) for a, b in zip(route[:-1], route[1:]))


def sign_changes(values, deadband=YAW_DEADBAND):
    """How often the sign flips, ignoring values within the deadband."""
    flips, last = 0, 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign and last and sign != last:
            flips += 1
        last = sign or last
    return flips


def metrics(samples, route):
    """Score a run. samples: [(t, x, y, (vx, vy, wz) measured body velocity, (vx, vy, wz) commanded
    or None)]; route: [(x, y)] from the start through every waypoint. Returns time, xte_mean,
    xte_max, final (m to the last waypoint), flips (yaw-rate sign changes), vmax, and track: per
    axis (mean, max) of |commanded - measured| over the samples that have a command."""
    if len(samples) < 2:
        raise ValueError('fewer than 2 odometry samples')
    errors = [cross_track((s[1], s[2]), route) for s in samples]
    _, xe, ye, _, _ = samples[-1]
    track = []
    for axis in range(3):
        diffs = [abs(s[4][axis] - s[3][axis]) for s in samples if s[4] is not None]
        track.append((sum(diffs) / len(diffs), max(diffs)) if diffs else None)
    return dict(time=samples[-1][0] - samples[0][0], xte_mean=sum(errors) / len(errors), xte_max=max(errors),
                final=math.hypot(xe - route[-1][0], ye - route[-1][1]),
                flips=sign_changes([s[3][2] for s in samples]),
                vmax=max(math.hypot(s[3][0], s[3][1]) for s in samples), track=track)


def params_hash(parameters):
    """Six hex digits of sorted name=value pairs: the same tuning gives the same hash."""
    text = '\n'.join(f'{name}={value!r}' for name, value in sorted(parameters.items()))
    return hashlib.sha256(text.encode()).hexdigest()[:6]


def summary(course, scale, result, status, ctrl):
    track = ' '.join('-' if a is None else f'{a[0]:.2f}/{a[1]:.2f}' for a in result.get('track') or [None] * 3)
    return (f"{course}{'' if scale == 1 else f' x{scale:g}'} {status} time={result['time']:.1f}s "
            f"xte={result['xte_mean']:.2f}/{result['xte_max']:.2f}m final={result['final']:.2f}m "
            f"flips={result['flips']} vmax={result['vmax']:.2f}m/s track={track} ctrl={ctrl}")


def verdict(reached, total):
    """'done' only when every leg was reached, else 'incomplete k/n'."""
    return 'done' if reached == total else f'incomplete {reached}/{total}'


def log_line(kind, message, when=None):
    """The day log's line format: HH:MM:SS, two spaces, the kind padded to 8, the message."""
    when = when or datetime.datetime.now()
    return f'{when:%H:%M:%S}  {kind:<8}{message}'


def append_log(line, when=None, data=None):
    """Append to $ASTRO_DATA/<YYYY-MM-DD>/log.txt; returns the path, or None without ASTRO_DATA."""
    data = data or os.environ.get('ASTRO_DATA')
    if not data:
        return None
    path = Path(data).expanduser() / f'{(when or datetime.date.today()):%Y-%m-%d}' / 'log.txt'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as handle:
        handle.write(line + '\n')
    return path


class Stop(Exception):
    """A stop signal (SIGINT, SIGTERM, SIGHUP) arrived; the running goal must be cancelled."""


STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def install_stop_handlers():
    """Raise Stop on the first SIGINT/SIGTERM/SIGHUP (later ones are ignored, so the cancel runs).

    rclpy's own SIGINT handler would shut the context down before the cancel could be sent, so the
    trial inits rclpy without it. Returns the previous handlers for ``restore_handlers``."""
    previous = {sig: signal.getsignal(sig) for sig in STOP_SIGNALS}
    fired = []

    def handler(signum, _frame):
        if not fired:
            fired.append(signum)
            raise Stop(signal.Signals(signum).name)
    for sig in STOP_SIGNALS:
        signal.signal(sig, handler)
    return previous


def restore_handlers(previous):
    for sig, handler in previous.items():
        signal.signal(sig, handler)


# --- ROS --------------------------------------------------------------------------------------

def course_markers(frame, start, poses, stamp):
    """MarkerArray of the course: the route as a line, each waypoint a numbered sphere."""
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    out = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    out.markers.append(clear)
    line = Marker()
    line.header.frame_id, line.header.stamp = frame, stamp
    line.ns, line.id, line.type, line.action = 'trial', 0, Marker.LINE_STRIP, Marker.ADD
    line.scale.x = 0.04
    line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.6, 0.1, 0.9
    line.pose.orientation.w = 1.0
    line.points = [Point(x=x, y=y, z=0.0) for x, y in [start[:2]] + [p[:2] for p in poses]]
    out.markers.append(line)
    for i, (x, y, _) in enumerate(poses, 1):
        for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = 'trial', 2 * i + (kind == Marker.TEXT_VIEW_FACING), kind, Marker.ADD
            m.pose.position.x, m.pose.position.y = x, y
            m.pose.position.z = 0.0 if kind == Marker.SPHERE else 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.1, 1.0
            if kind == Marker.TEXT_VIEW_FACING:
                m.text = str(i)
                m.color.r = m.color.g = m.color.b = 1.0
            out.markers.append(m)
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def controller_params(node, server='/controller_server', timeout=3.0):
    """{name: value} of the live controller_server parameters ({} when it does not answer)."""
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    listing = node.create_client(ListParameters, server + '/list_parameters')
    getter = node.create_client(GetParameters, server + '/get_parameters')
    try:
        if not (listing.wait_for_service(timeout_sec=timeout) and getter.wait_for_service(timeout_sec=timeout)):
            return {}
        future = listing.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            return {}
        names = list(future.result().result.names)
        future = getter.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            return {}
        return {n: parameter_value_to_python(v) for n, v in zip(names, future.result().values)}
    finally:
        node.destroy_client(listing)
        node.destroy_client(getter)


def run(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateThroughPoses
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

    from rclpy.signals import SignalHandlerOptions

    home = args.course == HOME
    if home:
        home_frame, home_pose = load_start()
        points, timeout, box = [], 120.0, None
    else:
        points, timeout, box = load_course(args.course, args.scale)
    box = box if args.box in (None, 'auto') else args.box
    box = parse_box(box) if box and box.lower() != 'none' else None
    previous = install_stop_handlers()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('trial')
    events = node.create_publisher(String, args.events, 10)
    markers = node.create_publisher(MarkerArray, '/trial/course', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom, cmd = {}, {}
    node.create_subscription(Odometry, args.odom, lambda m: odom.__setitem__('msg', m), qos_profile_sensor_data)
    node.create_subscription(Twist, args.cmd, lambda m: cmd.__setitem__('msg', (m, time.monotonic())), 10)
    client = ActionClient(node, NavigateThroughPoses, args.action)
    samples, reached, status = [], 0, None
    goal = {'sent': None}                  # the latest send_goal future: cancelled on any way out

    def cancel_active(wait=2.0):
        """Cancel the latest goal unless it has finished; wait up to ``wait`` s for the answer."""
        sent, goal['sent'] = goal['sent'], None
        if sent is None:
            return
        try:
            if not sent.done():                                     # stopped while sending: learn its handle
                rclpy.spin_until_future_complete(node, sent, timeout_sec=wait)
            handle = sent.result() if sent.done() else None
            if handle is None or not handle.accepted or handle.get_result_async().done():
                return
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=wait)
            print('trial: goal cancelled' if future.done() else 'trial: no cancel answer; check Nav2',
                  file=sys.stderr, flush=True)
        except Exception as error:
            print(f'trial: cancel failed ({error}); cancel the goal by hand', file=sys.stderr, flush=True)

    try:
        deadline = time.monotonic() + 10
        while 'msg' not in odom and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'msg' not in odom:
            raise SystemExit(f'trial: no odometry on {args.odom}')
        if not client.wait_for_server(timeout_sec=10):
            raise SystemExit(f'trial: no {args.action} action server (is navigation.launch up?)')
        ctrl = params_hash(controller_params(node)) if not args.no_hash else '-'
        first = odom['msg']
        frame = first.header.frame_id
        p = first.pose.pose
        start = (p.position.x, p.position.y, yaw_of(p.orientation))
        if home:
            if home_frame != frame:
                raise SystemExit(f'trial: the start pose is in {home_frame}, {args.odom} is in {frame}; '
                                 f'use --odom with the topic the trial ran on')
            poses = [home_pose]
            dx, dy = home_pose[0] - start[0], home_pose[1] - start[1]
            c, s = math.cos(start[2]), math.sin(start[2])
            points = [(c * dx + s * dy, -s * dx + c * dy)]
        else:
            poses = to_world(points, start)
            save_start(frame, start, args.course)
        route = [start[:2]] + [q[:2] for q in poses]
        centre = box_centre(points)
        markers.publish(course_markers(frame, start, poses, node.get_clock().now().to_msg()))
        print(f'trial {args.course} x{args.scale:g}: {len(poses)} legs in {frame}, timeout {timeout:.0f}s, '
              f'tolerance {args.tolerance:g}m, ctrl={ctrl}'
              + (f', box {box[0]:g}x{box[1]:g}m' if box else ''), flush=True)
        t0, printed, last, feedback = time.monotonic(), 0.0, None, {}
        try:
            for leg, (x, y, heading) in enumerate(poses, 1):
                request = NavigateThroughPoses.Goal()
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
                request.poses.append(pose)
                feedback.clear()
                sent = client.send_goal_async(
                    request, feedback_callback=lambda f: feedback.__setitem__('f', f.feedback))
                goal['sent'] = sent
                rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
                handle = sent.result() if sent.done() else None
                if handle is None or not handle.accepted:
                    status = f'rejected at leg {leg}'
                    break
                done = handle.get_result_async()
                while True:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    msg = odom['msg']
                    if msg is not last:
                        last = msg
                        q, v = msg.pose.pose.position, msg.twist.twist
                        c = cmd.get('msg')
                        commanded = (c[0].linear.x, c[0].linear.y, c[0].angular.z) if c and \
                            time.monotonic() - c[1] < 0.5 else None
                        samples.append((time.monotonic() - t0, q.x, q.y, (v.linear.x, v.linear.y, v.angular.z),
                                        commanded))
                    elapsed = time.monotonic() - t0
                    if elapsed - printed >= 1.0 and samples:
                        printed = elapsed
                        s = samples[-1]
                        to_go = math.hypot(x - s[1], y - s[2])
                        print(f'  {elapsed:5.1f}s  leg {leg}/{len(poses)}  {to_go:.1f}m to go  '
                              f'xte {cross_track(s[1:3], route):.2f}m  v {math.hypot(*s[3][:2]):.2f}m/s  '
                              f'wz {s[3][2]:+.2f}', flush=True)
                    if done.done():
                        break
                    why = box and samples and outside_box(samples[-1][1:3], start, box, centre)
                    if why:
                        status = why                # cancel_active below cancels the leg: Nav2 stops
                        print(f'trial: {why}; cancelling the goal', flush=True)
                        break
                    if elapsed > timeout:
                        status = 'timeout'          # cancel_active below cancels the leg
                        break
                if status:
                    break
                here = odom['msg'].pose.pose.position
                miss = math.hypot(x - here.x, y - here.y)
                if done.result().status != GoalStatus.STATUS_SUCCEEDED or miss > args.tolerance:
                    print(f'  leg {leg}: nav2 status {done.result().status}, {miss:.2f}m from its pose', flush=True)
                    break
                reached += 1
        except Stop as stop:
            status = f'abort ({stop})'
        cancel_active()
        status = f'{verdict(reached, len(poses))}' + (f' ({status})' if status else '')
        result = metrics(samples, route)
        message = summary(args.course, args.scale, result, status, ctrl)
        if box:
            message += f" margin={box_margin([q[1:3] for q in samples], start, box, centre):.2f}m"
        line = log_line(LOG_KIND, message)
        path = append_log(line)
        try:
            events.publish(String(data=line))
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception as error:          # the file has it
            print(f'trial: /events publish failed ({error})', file=sys.stderr)
        print(line + ('' if path else '   (ASTRO_DATA unset: not logged)'), flush=True)
        return 0 if reached == len(poses) else 1
    finally:
        cancel_active()                       # any way out (an error, a second signal): no goal left driving
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        restore_handlers(previous)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='trial', description='Drive a course (config/trials/COURSE.yaml, relative '
                                 'to the start pose) through Nav2 and print and log its metrics.')
    ap.add_argument('course', nargs='?', help='course name: ' + ', '.join(courses()))
    ap.add_argument('--list', action='store_true', help='print the courses, one per line')
    ap.add_argument('--scale', type=float, default=1.0, help='multiply the course size (e.g. 0.5 for a small area)')
    ap.add_argument('--odom', default='/odom/global', help='odometry topic; goals use its frame (default /odom/global)')
    ap.add_argument('--action', default='/navigate_through_poses', help='NavigateThroughPoses action')
    ap.add_argument('--cmd', default='/cmd_vel', help="Nav2's smoothed velocity command (default /cmd_vel)")
    ap.add_argument('--tolerance', type=float, default=0.5,
                    help='m from a leg pose that counts as reached (default 0.5, above the 0.4 goal checker)')
    ap.add_argument('--box', help='LENGTHxWIDTH m (length along the start heading) centred on the course: leaving '
                    "it cancels the goal and stops the trial (default: the course's box, pool courses 9x4; none: no box)")
    ap.add_argument('--events', default='/events', help='topic the log line is published on')
    ap.add_argument('--no-hash', action='store_true', help="skip reading controller_server's parameters")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)
    if args.list:
        print('\n'.join(courses()))
        return 0
    if not args.course:
        ap.error('a course is required (--list prints them)')

    try:
        return run(args)
    except ValueError as error:
        raise SystemExit(f'trial: {error}')
    except Stop as stop:
        raise SystemExit(f'trial: stopped ({stop}); no goal left running')


if __name__ == '__main__':
    sys.exit(main())
