from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    dataset_root = LaunchConfiguration("dataset_root")
    sequence = LaunchConfiguration("sequence")
    adas_ttc = LaunchConfiguration("adas_ttc_threshold")
    adas_fcw_rising = LaunchConfiguration("adas_publish_fcw_on_rising_edge_only")
    adas_ldw_rising = LaunchConfiguration("adas_publish_ldw_on_rising_edge_only")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root", default_value="/workspace/data/kitti"),
            DeclareLaunchArgument("sequence", default_value="0"),
            DeclareLaunchArgument("adas_ttc_threshold", default_value="2.5"),
            DeclareLaunchArgument("adas_publish_fcw_on_rising_edge_only", default_value="true"),
            DeclareLaunchArgument("adas_publish_ldw_on_rising_edge_only", default_value="true"),
            Node(
                package="dataset_replay_py",
                executable="kitti_replay_node",
                output="screen",
                parameters=[{"dataset_root": dataset_root, "sequence": sequence}],
            ),
            Node(package="perception_camera_py", executable="camera_perception_node", output="screen"),
            Node(package="lane_detection_py", executable="lane_detection_node", output="screen"),
            Node(package="perception_lidar_cpp", executable="lidar_perception_node", output="screen"),
            Node(package="fusion_tracker_cpp", executable="fusion_tracker_node", output="screen"),
            Node(
                package="adas_app_cpp",
                executable="adas_app_node",
                output="screen",
                parameters=[
                    {
                        "ttc_threshold": ParameterValue(adas_ttc, value_type=float),
                        "publish_fcw_on_rising_edge_only": ParameterValue(adas_fcw_rising, value_type=bool),
                        "publish_ldw_on_rising_edge_only": ParameterValue(adas_ldw_rising, value_type=bool),
                    }
                ],
            ),
        ]
    )
