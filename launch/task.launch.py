"""Run a task: ros2 launch mhseals_nav task.launch.py task:=pair_test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def task_node(context):
    arguments = ['--task', LaunchConfiguration('task').perform(context)]
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file:
        arguments += ['--params', params_file]
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    return [Node(package='mhseals_nav', executable='run_task', output='screen',
                 arguments=arguments, parameters=[{'use_sim_time': sim}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task',
                              description='task tree name in config/tasks/'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='task YAML; default config/tasks/<task>.yaml'),
        DeclareLaunchArgument('sim', default_value='false', description='use simulation time'),
        OpaqueFunction(function=task_node),
    ])
