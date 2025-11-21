from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    source_topic_arg = DeclareLaunchArgument(
        "source_topic",
        default_value="/camera/right/points",
        description="PointCloud2 topic for the source camera."
    )

    target_topic_arg = DeclareLaunchArgument(
        "target_topic",
        default_value="/camera/left/points",
        description="PointCloud2 topic for the target camera."
    )

    icp_rate_hz_arg = DeclareLaunchArgument(
        "icp_rate_hz",
        default_value="5.0",
        description="Maximum ICP update rate in Hz (CPU guard)."
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.01",
        description="VoxelGrid leaf size (meters); <= 0 disables downsampling."
    )

    fitness_score_threshold_arg = DeclareLaunchArgument(
        "fitness_score_threshold",
        default_value="0.5",
        description="Reject ICP results with fitness higher than this threshold."
    )

    use_prev_guess_arg = DeclareLaunchArgument(
        "use_previous_transform_as_initial_guess",
        default_value="true",
        description="Use previous ICP result as initial guess for the next iteration."
    )

    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Whether to publish the estimated transform into TF."
    )

    initial_tf_yaml_path_arg = DeclareLaunchArgument(
        "initial_tf_yaml_path",
        default_value="/home/mzahana/silki_ws/src/multi_cloud_fusion/config/camera12_initial_tf.yaml",
        description=(
            "Path to YAML file with initial transform "
            "(output of frame_tf_saver_node). Empty means identity."
        ),
    )

    # Node
    icp_tf_node = Node(
        package="multi_cloud_fusion",
        executable="icp_tf_node",
        name="icp_tf_node",
        output="screen",
        parameters=[
            {
                "source_topic": LaunchConfiguration("source_topic"),
                "target_topic": LaunchConfiguration("target_topic"),
                "icp_rate_hz": LaunchConfiguration("icp_rate_hz"),
                "voxel_leaf_size": LaunchConfiguration("voxel_leaf_size"),
                "fitness_score_threshold": LaunchConfiguration("fitness_score_threshold"),
                "use_previous_transform_as_initial_guess": LaunchConfiguration(
                    "use_previous_transform_as_initial_guess"
                ),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "initial_tf_yaml_path": LaunchConfiguration("initial_tf_yaml_path"),
            }
        ],
    )

    return LaunchDescription(
        [
            source_topic_arg,
            target_topic_arg,
            icp_rate_hz_arg,
            voxel_leaf_size_arg,
            fitness_score_threshold_arg,
            use_prev_guess_arg,
            publish_tf_arg,
            initial_tf_yaml_path_arg,
            icp_tf_node,
        ]
    )
