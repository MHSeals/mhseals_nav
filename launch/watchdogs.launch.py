"""The boat's hold-only safety watchdogs: operator link, geofence, FCU link and EKF inputs.

Each asks command_mux to hold while autonomy commands flow and never releases. Their settings
are config/<name>.yaml; an argument given here overrides one, and an empty one keeps the YAML.
A fence_points given here (the simulator's map-frame box) also skips fence_file, so an
operator's ~/.config/mhseals/fence.yaml cannot replace it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mhseals_nav.startup_projection import boolean
import yaml


def truth(name, text):
    """Parse the launch boolean ``text``, naming the argument when it is not one."""
    try:
        return boolean(text)
    except ValueError as error:
        raise ValueError(f'{name}: {error}') from None


def points(text):
    """fence_points as a flat list of floats (YAML or JSON text)."""
    try:
        values = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ValueError(f'fence_points: not a list ({error})') from None
    if not isinstance(values, list) or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) for v in values):
        raise ValueError(f'fence_points: a flat list of numbers, not {text!r}')
    return [float(v) for v in values]


def watchdog_parameters(value):
    """Return [(executable, parameter overrides)]; ``value(name)`` reads one launch argument."""
    common = {'use_sim_time': truth('sim', value('sim'))}
    link, fence = dict(common), dict(common)
    if value('require_operator'):
        link['require_operator'] = truth('require_operator', value('require_operator'))
    if value('fence_frame'):
        fence['fence_frame'] = value('fence_frame')
    if value('fence_points'):
        fence.update(fence_points=points(value('fence_points')), fence_file='')
    nodes = [('link_watchdog', link), ('fence_watchdog', fence)]
    if truth('fcu_watchdog', value('fcu_watchdog')):
        nodes.append(('fcu_watchdog', common))
    nodes.append(('ekf_input_watchdog', common))   # the simulator publishes its inputs too
    return nodes


def watchdogs(context):
    def value(name):
        return LaunchConfiguration(name).perform(context).strip()
    return [Node(package='mhseals_nav', executable=name, name=name, output='screen',
                 parameters=[parameters]) for name, parameters in watchdog_parameters(value)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('require_operator', default_value='',
                              description='link_watchdog holds before the first heartbeat'),
        DeclareLaunchArgument('fence_frame', default_value='', description='latlon or map'),
        DeclareLaunchArgument('fence_points', default_value='',
                              description='Flat [a, b, ...] vertex list; replaces fence_file'),
        DeclareLaunchArgument('fcu_watchdog', default_value='true',
                              description='Hold on a stale or disconnected MAVROS'),
        OpaqueFunction(function=watchdogs),
    ])
