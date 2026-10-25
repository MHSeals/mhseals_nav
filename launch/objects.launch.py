"""One semantic tracker; in sim it reads /detections, otherwise a converter feeds it."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    sim_value = LaunchConfiguration("sim").perform(context).lower()
    if sim_value not in ("true", "false"):
        raise ValueError("sim must be true or false")
    clock = ParameterValue(LaunchConfiguration("sim"), value_type=bool)
    remaps = [("detections", LaunchConfiguration("detections_topic"))]
    result = [
        Node(
            package="mhseals_nav",
            executable="object_tracker",
            parameters=[{"use_sim_time": clock}],
            remappings=remaps,
            output="screen",
        )
    ]
    if sim_value == "false":
        result.append(
            Node(
                package="mhseals_nav",
                executable="zed_detections_converter",
                parameters=[{"use_sim_time": clock, "objects_topic": "objects"}],
                remappings=remaps + [("objects", LaunchConfiguration("objects_topic"))],
                output="screen",
            )
        )
    return result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="true"),
            DeclareLaunchArgument("detections_topic", default_value="/detections"),
            DeclareLaunchArgument(
                "objects_topic", default_value="/front/zed_node/obj_det/objects"
            ),
            OpaqueFunction(function=nodes),
        ]
    )
