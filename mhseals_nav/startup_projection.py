"""Project final launch inputs before immutable bootstrap materialization."""
from copy import deepcopy
import hashlib
from pathlib import Path

SERVERS = (
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
)
BT_FILES = {'default_nav_to_pose_bt_xml': 'nav_to_pose.xml',
            'default_nav_through_poses_bt_xml': 'nav_replan_recov.xml'}


def resource_paths(share=None):
    """Select explicit installed resources, or source resources for pure callers."""
    root = Path(share) if share else Path(__file__).parents[1]
    trees = root / 'behavior_trees' if share else root / 'mhseals_nav/behavior_trees'
    return {'launch': str(root / 'launch/navigation.launch.py'),
            **{key: str(trees / name) for key, name in BT_FILES.items()}}


def resource_identity(paths):
    """Pin content plus installed and resolved paths of all selected resources."""
    if set(paths) != {'launch', *BT_FILES}:
        raise ValueError('incomplete navigation launch resource closure')
    return {key: {'path': str(Path(path).absolute()),
                  'resolved': str(Path(path).resolve(strict=True)),
                  'sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest()}
            for key, path in paths.items()}


def project_launch(parameters, invocation, resources):
    """Return the complete launch projection and requested/effective provenance."""
    result = deepcopy(parameters)
    changes = []

    def assign(mapping, key, value, path, reason):
        changes.append({'key': '.'.join((*path, key)),
                        'requested': deepcopy(mapping.get(key)),
                        'effective': value, 'reason': reason})
        mapping[key] = value

    def clocks(mapping, path=()):
        for key, value in mapping.items():
            if key == 'ros__parameters' and isinstance(value, dict):
                assign(value, 'use_sim_time', invocation['sim'], (*path, key),
                       'session simulation clock')
            elif isinstance(value, dict):
                clocks(value, (*path, key))

    clocks(result)
    selected = [(package, name) for package, name in SERVERS if name in result]
    for _, name in selected:
        params = result[name]['ros__parameters']
        assign(params, 'enable_stamped_cmd_vel', False, (name, 'ros__parameters'),
               'launch unstamped command contract')
    for name in ('local_costmap', 'global_costmap'):
        params = result.get(name, {}).get(name, {}).get('ros__parameters', {})
        if 'obstacle_layer' in params:
            assign(params['obstacle_layer'], 'enabled', invocation['use_lidar'],
                   (name, name, 'ros__parameters', 'obstacle_layer'),
                   'session lidar selection')
    if 'bt_navigator' in result:
        params = result['bt_navigator']['ros__parameters']
        for key in BT_FILES:
            assign(params, key, resources[key]['path'], ('bt_navigator', 'ros__parameters'),
                   'pinned installed behavior tree')
    lifecycle = {'autostart': True, 'use_sim_time': invocation['sim'],
                 'node_names': [name for _, name in selected]}
    result['lifecycle_manager_navigation'] = {'ros__parameters': lifecycle}
    return result, changes
