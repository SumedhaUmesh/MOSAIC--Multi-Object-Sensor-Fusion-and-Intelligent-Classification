from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    dataset_root = LaunchConfiguration("dataset_root")
    sequence = LaunchConfiguration("sequence")
    adas_ttc = LaunchConfiguration("adas_ttc_threshold")
    adas_fcw_rising = LaunchConfiguration("adas_publish_fcw_on_rising_edge_only")
    adas_ldw_rising = LaunchConfiguration("adas_publish_ldw_on_rising_edge_only")
    foxglove_port = LaunchConfiguration("foxglove_port")
    fusion_prediction_dt = LaunchConfiguration("fusion_prediction_dt")
    fusion_max_assignment_cost = LaunchConfiguration("fusion_max_assignment_cost")
    fusion_mahalanobis_gate = LaunchConfiguration("fusion_mahalanobis_gate")
    fusion_iou_weight = LaunchConfiguration("fusion_iou_weight")
    fusion_confirm_hits = LaunchConfiguration("fusion_confirm_hits")
    fusion_tentative_max_misses = LaunchConfiguration("fusion_tentative_max_misses")
    fusion_confirmed_max_misses = LaunchConfiguration("fusion_confirmed_max_misses")

    mosaic_defaults = ParameterFile(
        PathJoinSubstitution([FindPackageShare("mosaic_bringup"), "config", "mosaic_defaults.yaml"]),
        allow_substs=False,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root", default_value="/workspace/data/kitti"),
            DeclareLaunchArgument("sequence", default_value="0"),
            DeclareLaunchArgument("adas_ttc_threshold", default_value="2.5"),
            DeclareLaunchArgument("adas_publish_fcw_on_rising_edge_only", default_value="true"),
            DeclareLaunchArgument("adas_publish_ldw_on_rising_edge_only", default_value="true"),
            DeclareLaunchArgument(
                "launch_foxglove_bridge",
                default_value="false",
                description="If true, start foxglove_bridge (WebSocket on foxglove_port for Foxglove Studio).",
            ),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("fusion_prediction_dt", default_value="0.1"),
            DeclareLaunchArgument("fusion_max_assignment_cost", default_value="12.0"),
            DeclareLaunchArgument("fusion_mahalanobis_gate", default_value="9.21"),
            DeclareLaunchArgument("fusion_iou_weight", default_value="2.0"),
            DeclareLaunchArgument("fusion_confirm_hits", default_value="3"),
            DeclareLaunchArgument("fusion_tentative_max_misses", default_value="2"),
            DeclareLaunchArgument("fusion_confirmed_max_misses", default_value="8"),
            Node(
                package="dataset_replay_py",
                executable="kitti_replay_node",
                output="screen",
                parameters=[
                    mosaic_defaults,
                    {"dataset_root": dataset_root, "sequence": sequence},
                ],
            ),
            Node(
                package="perception_camera_py",
                executable="camera_perception_node",
                output="screen",
                parameters=[mosaic_defaults],
            ),
            Node(
                package="lane_detection_py",
                executable="lane_detection_node",
                output="screen",
                parameters=[mosaic_defaults],
            ),
            Node(
                package="perception_lidar_cpp",
                executable="lidar_perception_node",
                output="screen",
                parameters=[mosaic_defaults],
            ),
            Node(
                package="fusion_tracker_cpp",
                executable="fusion_tracker_node",
                output="screen",
                parameters=[
                    mosaic_defaults,
                    {
                        "prediction_dt": ParameterValue(fusion_prediction_dt, value_type=float),
                        "max_assignment_cost": ParameterValue(fusion_max_assignment_cost, value_type=float),
                        "mahalanobis_gate": ParameterValue(fusion_mahalanobis_gate, value_type=float),
                        "iou_weight": ParameterValue(fusion_iou_weight, value_type=float),
                        "confirm_hits": ParameterValue(fusion_confirm_hits, value_type=int),
                        "tentative_max_misses": ParameterValue(fusion_tentative_max_misses, value_type=int),
                        "confirmed_max_misses": ParameterValue(fusion_confirmed_max_misses, value_type=int),
                    },
                ],
            ),
            Node(
                package="adas_app_cpp",
                executable="adas_app_node",
                output="screen",
                parameters=[
                    mosaic_defaults,
                    {
                        "ttc_threshold": ParameterValue(adas_ttc, value_type=float),
                        "publish_fcw_on_rising_edge_only": ParameterValue(adas_fcw_rising, value_type=bool),
                        "publish_ldw_on_rising_edge_only": ParameterValue(adas_ldw_rising, value_type=bool),
                    },
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("launch_foxglove_bridge"), "' == 'true'"])
                ),
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    mosaic_defaults,
                    {"port": ParameterValue(foxglove_port, value_type=int)},
                    {"address": "0.0.0.0"},
                ],
            ),
        ]
    )
