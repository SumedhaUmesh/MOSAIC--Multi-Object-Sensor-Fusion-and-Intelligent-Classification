from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dataset_root = LaunchConfiguration("dataset_root")
    sequence = LaunchConfiguration("sequence")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root", default_value="/workspace/data/kitti"),
            DeclareLaunchArgument("sequence", default_value="0"),
            Node(
                package="dataset_replay_py",
                executable="kitti_replay_node",
                output="screen",
                parameters=[{"dataset_root": dataset_root, "sequence": sequence}],
            ),
            Node(package="perception_camera_py", executable="camera_perception_node", output="screen"),
            Node(package="perception_lidar_cpp", executable="lidar_perception_node", output="screen"),
            Node(package="fusion_tracker_cpp", executable="fusion_tracker_node", output="screen"),
            Node(package="adas_app_cpp", executable="adas_app_node", output="screen"),
        ]
    )
