"""One semantic tracker; native/sim detections or a legacy ZED adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    source = LaunchConfiguration("source").perform(context)
    if source not in ("detections", "zed", "bridge"):
        raise ValueError("source must be detections, zed, or bridge")
    clock = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
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
    if source != "detections":
        url = LaunchConfiguration("rosbridge_url").perform(context)
        if source == "bridge" and not url:
            raise ValueError("bridge requires rosbridge_url:=ws://host:9090")
        result.append(
            Node(
                package="mhseals_nav",
                executable="zed_detections_converter",
                parameters=[
                    {
                        "use_sim_time": clock,
                        "objects_topic": LaunchConfiguration("objects_topic"),
                        "camera_root_frame": LaunchConfiguration("camera_root_frame"),
                        "rosbridge_url": url if source == "bridge" else "",
                    }
                ],
                remappings=remaps,
                output="screen",
            )
        )
    return result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("source", default_value="detections"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("detections_topic", default_value="/detections"),
            DeclareLaunchArgument(
                "objects_topic", default_value="/front/zed_node/obj_det/objects"
            ),
            DeclareLaunchArgument("rosbridge_url", default_value=""),
            DeclareLaunchArgument("camera_root_frame", default_value=""),
            OpaqueFunction(function=nodes),
        ]
    )
