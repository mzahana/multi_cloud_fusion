from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # -------- Launch Configurations --------
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_frame = LaunchConfiguration('target_frame')
    
    # Topic configurations for remapping
    cloud1_topic = LaunchConfiguration('cloud1_topic')
    cloud2_topic = LaunchConfiguration('cloud2_topic')
    fused_topic = LaunchConfiguration('fused_topic')

    return LaunchDescription([
        # -------- Global args --------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='morpho_conveyor_base',
            description='Target frame for fused cloud'
        ),

        # -------- Topic Remapping args --------
        DeclareLaunchArgument(
            'cloud1_topic',
            default_value='/camera/left/points',
            description='Input topic for the first point cloud'
        ),
        DeclareLaunchArgument(
            'cloud2_topic',
            default_value='/camera/right/points',
            description='Input topic for the second point cloud'
        ),
        DeclareLaunchArgument(
            'fused_topic',
            default_value='/fused_cloud',
            description='Output topic for the merged point cloud'
        ),

        # -------- Cloud Fusion Node --------
        Node(
            package='multi_cloud_fusion',
            executable='cloud_fusion_node',
            name='cloud_fusion_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'target_frame': target_frame},
            ],
            remappings=[
                # syntax: ('internal_node_topic', 'external_ros_topic')
                ('cloud1', cloud1_topic),
                ('cloud2', cloud2_topic),
                ('fused_cloud', fused_topic),
            ],
        ),
    ])