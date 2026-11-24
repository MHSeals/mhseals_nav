"""trial: drive a course through Nav2 (NavigateThroughPoses) and score it.

  ros2 run mhseals_nav trial COURSE [--scale S]

A course is config/trials/COURSE.yaml: ``points`` ([x, y] metres in the start pose's frame, x ahead,
y left) and ``timeout_s``. The trial reads the start pose from odometry and drives the course one
leg per NavigateThroughPoses goal: a closed course ends where it starts, and Nav2's goal checker
looks only at a path's last pose, so one goal for the whole loop "arrives" at once. A leg counts
as reached when Nav2 succeeds and the boat is within --tolerance of its pose; the trial is
``done`` only when every leg is, else ``incomplete k/n``. It prints a progress line every second,
then the metrics (time, mean/max cross-track error, final position error, yaw-rate sign changes,
max speed, and the command-vs-actual tracking error per axis). The summary is appended to $ASTRO_DATA/<date>/log.txt
(``HH:MM:SS  trial   ...``) and published on /events, with a short hash of controller_server's live
parameters so two trials can be told apart by tuning. Ctrl-C cancels the goal.
"""

import argparse
import datetime
import hashlib
import math
import os
from pathlib import Path
import sys
import time

from mhseals_nav.layers import config_path, load_strict

LOG_KIND = 'trial'
YAW_DEADBAND = 0.05     # rad/s: yaw rates inside this are not a sign (noise around zero)


# --- pure -------------------------------------------------------------------------------------

def courses():
    return sorted(p.stem for p in config_path('trials').glob('*.yaml'))


def load_course(name, scale=1.0, path=None):
    """(points [(x, y)] in the start frame, scaled; timeout_s) of config/trials/<name>.yaml."""
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
    return [(float(x) * scale, float(y) * scale) for x, y in points], float(data.get('timeout_s', 300)) * max(scale, 1.0)


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


# --- ROS --------------------------------------------------------------------------------------

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
    from std_msgs.msg import String

    points, timeout = load_course(args.course, args.scale)
    rclpy.init()
    node = rclpy.create_node('trial')
    events = node.create_publisher(String, args.events, 10)
    odom, cmd = {}, {}
    node.create_subscription(Odometry, args.odom, lambda m: odom.__setitem__('msg', m), qos_profile_sensor_data)
    node.create_subscription(Twist, args.cmd, lambda m: cmd.__setitem__('msg', (m, time.monotonic())), 10)
    client = ActionClient(node, NavigateThroughPoses, args.action)
    samples, reached, status = [], 0, None
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
        poses = to_world(points, start)
        route = [start[:2]] + [q[:2] for q in poses]
        print(f'trial {args.course} x{args.scale:g}: {len(poses)} legs in {frame}, timeout {timeout:.0f}s, '
              f'tolerance {args.tolerance:g}m, ctrl={ctrl}', flush=True)
        t0, printed, last, feedback = time.monotonic(), 0.0, None, {}
        for leg, (x, y, heading) in enumerate(poses, 1):
            goal = NavigateThroughPoses.Goal()
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
            goal.poses.append(pose)
            feedback.clear()
            sent = client.send_goal_async(goal, feedback_callback=lambda f: feedback.__setitem__('f', f.feedback))
            rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
            handle = sent.result() if sent.done() else None
            if handle is None or not handle.accepted:
                status = f'rejected at leg {leg}'
                break
            done = handle.get_result_async()
            try:
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
                    if elapsed > timeout:
                        handle.cancel_goal_async()
                        status = 'timeout'
                        break
            except KeyboardInterrupt:
                status = 'abort (ctrl-c)'
                try:
                    future = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                except Exception as error:      # the context may already be shutting down
                    print(f'trial: cancel failed ({error}); cancel the goal by hand', file=sys.stderr)
            if status:
                break
            here = odom['msg'].pose.pose.position
            miss = math.hypot(x - here.x, y - here.y)
            if done.result().status != GoalStatus.STATUS_SUCCEEDED or miss > args.tolerance:
                print(f'  leg {leg}: nav2 status {done.result().status}, {miss:.2f}m from its pose', flush=True)
                break
            reached += 1
        status = f'{verdict(reached, len(poses))}' + (f' ({status})' if status else '')
        result = metrics(samples, route)
        message = summary(args.course, args.scale, result, status, ctrl)
        line = log_line(LOG_KIND, message)
        path = append_log(line)
        try:
            events.publish(String(data=line))
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception as error:          # after Ctrl-C the context may be gone; the file has it
            print(f'trial: /events publish failed ({error})', file=sys.stderr)
        print(line + ('' if path else '   (ASTRO_DATA unset: not logged)'), flush=True)
        return 0 if reached == len(poses) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(prog='trial', description='Drive a course (config/trials/COURSE.yaml, relative '
                                 'to the start pose) through Nav2 and print and log its metrics.')
    ap.add_argument('course', help='course name: ' + ', '.join(courses()))
    ap.add_argument('--scale', type=float, default=1.0, help='multiply the course size (e.g. 0.5 for a small area)')
    ap.add_argument('--odom', default='/odom/global', help='odometry topic; goals use its frame (default /odom/global)')
    ap.add_argument('--action', default='/navigate_through_poses', help='NavigateThroughPoses action')
    ap.add_argument('--cmd', default='/cmd_vel', help="Nav2's smoothed velocity command (default /cmd_vel)")
    ap.add_argument('--tolerance', type=float, default=0.5,
                    help='m from a leg pose that counts as reached (default 0.5, above the 0.4 goal checker)')
    ap.add_argument('--events', default='/events', help='topic the log line is published on')
    ap.add_argument('--no-hash', action='store_true', help="skip reading controller_server's parameters")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return run(args)
    except ValueError as error:
        raise SystemExit(f'trial: {error}')


if __name__ == '__main__':
    sys.exit(main())
