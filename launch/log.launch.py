from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from datetime import datetime

def generate_launch_description():
    bag_filename = LaunchConfiguration('bag_filename')
    mode = LaunchConfiguration('mode')

    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        default_value='bags/' + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
        description='Name of rosbag file'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value="log",
        description='Log or play'
    )
    
    bag_log_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_filename],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'log'"
            ])
        )
    )

    bag_play_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_filename],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'play'"
            ])
        )
    )

    return LaunchDescription([
        bag_filename_arg,
        mode_arg,
        bag_log_process,
        bag_play_process
    ])