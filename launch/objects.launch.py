"""One semantic tracker; native/sim detections or a legacy ZED adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    sim_value = LaunchConfiguration("sim").perform(context).lower()
    if sim_value not in ("true", "false"):
        raise ValueError("sim must be true or false")
    sim = sim_value == "true"
    url = LaunchConfiguration("rosbridge_url").perform(context)
    root = LaunchConfiguration("camera_root_frame").perform(context)
    if url and sim:
        raise ValueError("rosbridge_url requires sim:=false")
    if url and not root:
        raise ValueError("legacy bridge requires camera_root_frame")
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
    if not sim:
        result.append(
            Node(
                package="mhseals_nav",
                executable="zed_detections_converter",
                parameters=[
                    {
                        "use_sim_time": clock,
                        "objects_topic": (
                            LaunchConfiguration("objects_topic") if url else "objects"
                        ),
                        "camera_root_frame": LaunchConfiguration("camera_root_frame"),
                        "rosbridge_url": url,
                    }
                ],
                remappings=remaps + ([] if url else [
                    ("objects", LaunchConfiguration("objects_topic"))
                ]),
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
            DeclareLaunchArgument("rosbridge_url", default_value=""),
            DeclareLaunchArgument("camera_root_frame", default_value=""),
            OpaqueFunction(function=nodes),
        ]
    )
