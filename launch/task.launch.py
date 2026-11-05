"""Run a task: ros2 launch mhseals_nav task.launch.py task:=pair_test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def task_node(context):
    task = LaunchConfiguration('task').perform(context)
    if not task:   # a whole-profile bring-up (device.launch.py device:=sim) runs no task
        return [LogInfo(msg='task.launch: no task:=, run_task not started')]
    arguments = ['--task', task]
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file:
        arguments += ['--params', params_file]
    from mhseals_nav.endpoint_configuration import boolean
    sim = boolean(LaunchConfiguration('sim').perform(context))
    return [Node(package='mhseals_nav', executable='run_task', output='screen',
                 arguments=arguments, parameters=[{'use_sim_time': sim}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='',
                              description='task tree name in config/tasks/; empty runs none'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='task YAML; default config/tasks/<task>.yaml'),
        DeclareLaunchArgument('sim', default_value='false', description='use simulation time'),
        OpaqueFunction(function=task_node),
    ])
