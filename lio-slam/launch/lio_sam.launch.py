import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('lio_sam_custom')
    
    # Define the path to your config file
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')
    
    # Declare the 'use_sim_time' launch argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # LIO-SAM node that loads parameters from the YAML file
    lio_sam_node = Node(
        package='lio_sam_custom',
        executable='lio_sam_custom_node',
        name='lio_sam_node',
        output='screen',
        parameters=[
            config_file,  # <-- THIS IS THE FIX. It loads all params from your YAML.
            {'use_sim_time': LaunchConfiguration('use_sim_time')} # You can still override specific params.
        ],
        remappings=[
            ('/lidar/velodyne_points', '/lidar/velodyne_points'),
            ('/imu/data', '/imu/data'),
        ]
    )
    
    # RViz node for visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lio_sam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Static transform from base_link to lidar
    # static_tf_lidar = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_lidar',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    #
    # # Static transform from base_link to imu
    # static_tf_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_imu',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return LaunchDescription([
        use_sim_time_arg,
        lio_sam_node,
        rviz_node,
        # static_tf_lidar,
        # static_tf_imu
    ])
