"""Per-goal timing and in-place rotation measured from a sim mission bag (/cmd_vel_nav bursts)."""
from pathlib import Path

import yaml

GAP_S = 0.8        # a longer pause in /cmd_vel_nav starts a new goal
MIN_GOAL_S = 2.0   # shorter bursts are not goals
LINEAR_EPS = 0.02
YAW_EPS = 0.05


def split_goals(cmds):
    """Group (t, vx, vy, wz) samples into goals; return a list of sample lists."""
    goals = []
    for sample in cmds:
        if goals and sample[0] - goals[-1][-1][0] < GAP_S:
            goals[-1].append(sample)
        else:
            goals.append([sample])
    return [g for g in goals if g[-1][0] - g[0][0] > MIN_GOAL_S]


def goal_stats(goal):
    """Return {'duration', 'rotate', 'reverse'}: seconds, seconds of pure rotation, fraction reversing."""
    duration = goal[-1][0] - goal[0][0]
    step = duration / len(goal)
    rotate = sum(step for _, vx, vy, wz in goal
                 if abs(vx) < LINEAR_EPS and abs(vy) < LINEAR_EPS and abs(wz) > YAW_EPS)
    reverse = sum(1 for _, vx, _, _ in goal if vx < -LINEAR_EPS) / len(goal)
    return {'duration': duration, 'rotate': rotate, 'reverse': reverse}


def violations(cmds, gates):
    """List the gate violations of a mission; empty means it passes."""
    stats = [goal_stats(g) for g in split_goals(cmds)]
    found = []
    if len(stats) != gates['goals']:
        found.append(f"expected {gates['goals']} goals, found {len(stats)}")
    for i, s in enumerate(stats, 1):
        limit = gates['max_goal_s'][i - 1] if i <= len(gates['max_goal_s']) else max(gates['max_goal_s'])
        if s['duration'] > limit:
            found.append(f"goal {i}: {s['duration']:.0f} s > {limit} s")
        if s['rotate'] > gates['max_rotate_s']:
            found.append(f"goal {i}: {s['rotate']:.0f} s in-place rotation > {gates['max_rotate_s']} s")
        if s['reverse'] > gates['max_reverse_frac']:
            found.append(f"goal {i}: reversing {s['reverse']:.0%} of the time > {gates['max_reverse_frac']:.0%}")
    total = sum(s['duration'] for s in stats)
    if total > gates['max_total_s']:
        found.append(f"total {total:.0f} s > {gates['max_total_s']} s")
    return found


def load_gates(path):
    """Read the gate YAML."""
    return yaml.safe_load(Path(path).read_text())


def read_cmds(bag):
    """Read (t, vx, vy, wz) from /cmd_vel_nav in a rosbag2 directory (needs ROS)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/cmd_vel_nav']))
    out = []
    while reader.has_next():
        _, data, t = reader.read_next()
        m = deserialize_message(data, get_message(types['/cmd_vel_nav']))
        out.append((t * 1e-9, m.linear.x, m.linear.y, m.angular.z))
    return out
