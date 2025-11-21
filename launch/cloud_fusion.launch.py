from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------- Launch arguments (so you can tweak from CLI) --------
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_frame = LaunchConfiguration('target_frame')

    use_voxel_grid = LaunchConfiguration('use_voxel_grid')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size')

    use_z_crop = LaunchConfiguration('use_z_crop')
    z_min = LaunchConfiguration('z_min')
    z_max = LaunchConfiguration('z_max')

    use_outlier_removal = LaunchConfiguration('use_outlier_removal')
    outlier_radius = LaunchConfiguration('outlier_radius')
    outlier_min_neighbors = LaunchConfiguration('outlier_min_neighbors')

    return LaunchDescription([
        # -------- Global-ish args --------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='morpho_conveyor_base',
            description='Target frame for fused cloud',
        ),

        # -------- Voxel grid params --------
        DeclareLaunchArgument(
            'use_voxel_grid',
            default_value='false',
            description='Enable PCL VoxelGrid downsampling',
        ),
        DeclareLaunchArgument(
            'voxel_leaf_size',
            default_value='0.005',
            description='Voxel leaf size in meters (used if use_voxel_grid=true)',
        ),

        # -------- Z crop params --------
        DeclareLaunchArgument(
            'use_z_crop',
            default_value='false',
            description='Enable z-axis cropping of fused cloud',
        ),
        DeclareLaunchArgument(
            'z_min',
            default_value='-1.0',
            description='Minimum z (meters) to keep when use_z_crop=true',
        ),
        DeclareLaunchArgument(
            'z_max',
            default_value='2.0',
            description='Maximum z (meters) to keep when use_z_crop=true',
        ),

        # -------- Outlier removal params --------
        DeclareLaunchArgument(
            'use_outlier_removal',
            default_value='false',
            description='Enable radius-based outlier removal',
        ),
        DeclareLaunchArgument(
            'outlier_radius',
            default_value='0.01',
            description='Radius (meters) used for outlier removal',
        ),
        DeclareLaunchArgument(
            'outlier_min_neighbors',
            default_value='5',
            description='Minimum neighbors in radius to keep a point',
        ),

        # -------- Cloud fusion node --------
        Node(
            package='multi_cloud_fusion',
            executable='cloud_fusion_node',
            name='cloud_fusion_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'target_frame': target_frame},

                # filters
                {'use_voxel_grid': use_voxel_grid},
                {'voxel_leaf_size': voxel_leaf_size},

                {'use_z_crop': use_z_crop},
                {'z_min': z_min},
                {'z_max': z_max},

                {'use_outlier_removal': use_outlier_removal},
                {'outlier_radius': outlier_radius},
                {'outlier_min_neighbors': outlier_min_neighbors},
            ],
            remappings=[
                # remap internal topic names to your camera topics
                ('cloud1', '/camera/left/points'),
                ('cloud2', '/camera/right/points'),
                # fused output (optional remap)
                ('fused_cloud', '/fused_cloud'),
            ],
        ),
    ])
