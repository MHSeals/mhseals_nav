"""The boat's hold-only safety watchdogs: operator link, geofence, FCU link and EKF inputs.

Each asks command_mux to hold while autonomy commands flow. Their settings are their sections of
config/watchdogs.yaml; an argument given here overrides one, and an empty one keeps the YAML
(fence_file: the simulator's fences/crane.yaml instead of the operator's file).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mhseals_nav.startup_projection import boolean


def truth(name, text):
    """Parse the launch boolean ``text``, naming the argument when it is not one."""
    try:
        return boolean(text)
    except ValueError as error:
        raise ValueError(f'{name}: {error}') from None


def watchdog_parameters(value):
    """Return [(executable, parameter overrides)]; ``value(name)`` reads one launch argument."""
    common = {'use_sim_time': truth('sim', value('sim'))}
    link, fence = dict(common), dict(common)
    if value('require_operator'):
        link['require_operator'] = truth('require_operator', value('require_operator'))
    if value('fence_file'):
        fence['fence_file'] = value('fence_file')
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
        DeclareLaunchArgument('fence_file', default_value='',
                              description='The fence file (relative: in config/)'),
        DeclareLaunchArgument('fcu_watchdog', default_value='true',
                              description='Hold on a stale or disconnected MAVROS'),
        OpaqueFunction(function=watchdogs),
    ])
