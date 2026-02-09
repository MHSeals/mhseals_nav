from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    urdf_path = "/home/roboboat/roboboat_ws/src/crane_control/description/omni_boat/omni_boat.urdf"
    thruster_map = "/home/roboboat/roboboat_ws/src/crane_control/config/thrusters.yaml"
    return LaunchDescription([
        Node(
            package="crane_control",
            executable="thruster_manager",
            name="thruster_manager",
            parameters=[
                {"robot_description": urdf_path},
                {"thruster_map": thruster_map}
            ]
        )
    ])