from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(package="dataset_replay_py", executable="kitti_replay_node", output="screen"),
            Node(package="perception_camera_py", executable="camera_perception_node", output="screen"),
            Node(package="perception_lidar_cpp", executable="lidar_perception_node", output="screen"),
            Node(package="fusion_tracker_cpp", executable="fusion_tracker_node", output="screen"),
            Node(package="adas_app_cpp", executable="adas_app_node", output="screen"),
        ]
    )
