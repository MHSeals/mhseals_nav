from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from datetime import datetime

def generate_launch_description():
    launch_args = [
        ('bag_filename', 'bags/' + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), 'Name of rosbag file'),
        ('mode', 'log', 'Log or play')
    ]

    launch_configurations = {}
    declare_arguments = []
    for name, default_value, description in launch_args:
        launch_configurations[name] = LaunchConfiguration(name)
        declare_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Processes
    bag_log_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', launch_configurations['bag_filename']],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([launch_configurations['mode'], " == 'log'"])
        )
    )
    
    bag_play_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', launch_configurations['bag_filename']],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([launch_configurations['mode'], " == 'play'"])
        )
    )

    return LaunchDescription(declare_arguments + [bag_log_process, bag_play_process])