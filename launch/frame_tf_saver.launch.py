from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    source_frame_arg = DeclareLaunchArgument(
        "source_frame",
        default_value="morpho_conveyor/left_camera/link/realsense_d435",
        description="Source frame (child frame) of the transform."
    )

    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="morpho_conveyor/right_camera/link/realsense_d435",
        description="Target frame (parent frame) of the transform."
    )

    output_yaml_path_arg = DeclareLaunchArgument(
        "output_yaml_path",
        default_value="/home/mzahana/silki_ws/src/multi_cloud_fusion/config/camera12_initial_tf.yaml",
        description="Path to YAML file where the transform will be saved."
    )

    lookup_timeout_sec_arg = DeclareLaunchArgument(
        "lookup_timeout_sec",
        default_value="10.0",
        description="Maximum time in seconds to wait for TF before giving up."
    )

    # Node
    frame_tf_saver_node = Node(
        package="multi_cloud_fusion",
        executable="frame_tf_saver_node",
        name="frame_tf_saver_node",
        output="screen",
        parameters=[
            {
                "source_frame": LaunchConfiguration("source_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "output_yaml_path": LaunchConfiguration("output_yaml_path"),
                "lookup_timeout_sec": LaunchConfiguration("lookup_timeout_sec"),
            }
        ],
    )

    return LaunchDescription(
        [
            source_frame_arg,
            target_frame_arg,
            output_yaml_path_arg,
            lookup_timeout_sec_arg,
            frame_tf_saver_node,
        ]
    )
