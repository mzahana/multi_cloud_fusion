from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Launch arguments (Configurations) ----
    
    # Topic Remapping Configurations
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    output_rgb_topic = LaunchConfiguration('output_rgb_topic')
    output_depth_topic = LaunchConfiguration('output_depth_topic')
    output_index_map_topic = LaunchConfiguration('output_index_map_topic')

    # General Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    x_min = LaunchConfiguration('x_min')
    x_max = LaunchConfiguration('x_max')
    y_min = LaunchConfiguration('y_min')
    y_max = LaunchConfiguration('y_max')
    use_max_height = LaunchConfiguration('use_max_height')
    bg_r = LaunchConfiguration('bg_r')
    bg_g = LaunchConfiguration('bg_g')
    bg_b = LaunchConfiguration('bg_b')

    # Filter-related Configurations
    use_hole_filling = LaunchConfiguration('use_hole_filling')
    hole_filling_iterations = LaunchConfiguration('hole_filling_iterations')
    use_smoothing = LaunchConfiguration('use_smoothing')
    smoothing_kernel_size = LaunchConfiguration('smoothing_kernel_size')

    return LaunchDescription([
        # ---- Topic Arguments ----
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='/fused_cloud',
            description='Input PointCloud2 topic to project',
        ),
        DeclareLaunchArgument(
            'output_rgb_topic',
            default_value='/top_view/rgb',
            description='Topic name for the output RGB top-view image',
        ),
        DeclareLaunchArgument(
            'output_depth_topic',
            default_value='/top_view/depth',
            description='Topic name for the output Depth top-view image',
        ),
        DeclareLaunchArgument(
            'output_index_map_topic',
            default_value='/top_view/index_map',
            description='Topic name for the output Index Map',
        ),

        # ---- Generic args ----
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation /clock if true',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Top-view image width in pixels',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Top-view image height in pixels',
        ),
        DeclareLaunchArgument(
            'x_min',
            default_value='-1.0',
            description='Min X (m) of ROI in cloud frame',
        ),
        DeclareLaunchArgument(
            'x_max',
            default_value='1.0',
            description='Max X (m) of ROI in cloud frame',
        ),
        DeclareLaunchArgument(
            'y_min',
            default_value='-1.0',
            description='Min Y (m) of ROI in cloud frame',
        ),
        DeclareLaunchArgument(
            'y_max',
            default_value='1.0',
            description='Max Y (m) of ROI in cloud frame',
        ),
        DeclareLaunchArgument(
            'use_max_height',
            default_value='true',
            description='If true, take highest z per pixel; else lowest',
        ),
        DeclareLaunchArgument(
            'bg_r',
            default_value='0',
            description='Background R channel (0-255)',
        ),
        DeclareLaunchArgument(
            'bg_g',
            default_value='0',
            description='Background G channel (0-255)',
        ),
        DeclareLaunchArgument(
            'bg_b',
            default_value='0',
            description='Background B channel (0-255)',
        ),

        # ---- Filter args ----
        DeclareLaunchArgument(
            'use_hole_filling',
            default_value='true',
            description='Enable simple 2D hole filling on RGB/depth',
        ),
        DeclareLaunchArgument(
            'hole_filling_iterations',
            default_value='1',
            description='Number of hole-filling iterations (>=1)',
        ),
        DeclareLaunchArgument(
            'use_smoothing',
            default_value='false',
            description='Enable 2D smoothing (box filter) on RGB/depth',
        ),
        DeclareLaunchArgument(
            'smoothing_kernel_size',
            default_value='3',
            description='Box filter kernel size (odd, >=3)',
        ),

        # ---- Node ----
        Node(
            package='multi_cloud_fusion',
            executable='top_view_projector_node',
            name='top_view_projector_node',
            output='screen',
            remappings=[
                ('cloud', input_cloud_topic),
                ('top_view/rgb', output_rgb_topic),
                ('top_view/depth', output_depth_topic),
                ('/top_view/index_map', output_index_map_topic),
            ],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'image_width': image_width},
                {'image_height': image_height},
                {'x_min': x_min},
                {'x_max': x_max},
                {'y_min': y_min},
                {'y_max': y_max},
                {'use_max_height': use_max_height},
                {'bg_r': bg_r},
                {'bg_g': bg_g},
                {'bg_b': bg_b},
                {'use_hole_filling': use_hole_filling},
                {'hole_filling_iterations': hole_filling_iterations},
                {'use_smoothing': use_smoothing},
                {'smoothing_kernel_size': smoothing_kernel_size},
            ],
        ),
    ])