#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='6.0',
        description='Size of the costmap grid in meters'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Resolution of the costmap in meters per cell'
    )
    
    max_height_diff_arg = DeclareLaunchArgument(
        'max_height_diff',
        default_value='0.3',
        description='Maximum height difference for traversable terrain'
    )
    
    # Stereo costmap node
    stereo_costmap_node = Node(
        package='local_costmap',
        executable='stereo_costmap_node',
        name='stereo_costmap',
        output='screen',
        parameters=[{
            'grid_size': LaunchConfiguration('grid_size'),
            'resolution': LaunchConfiguration('resolution'),
            'max_height_diff': LaunchConfiguration('max_height_diff'),
        }],
        remappings=[
            ('/stereo/left/image_raw', '/camera/left/image_raw'),
            ('/stereo/right/image_raw', '/camera/right/image_raw'),
            ('/stereo/left/camera_info', '/camera/left/camera_info'),
            ('/stereo/right/camera_info', '/camera/right/camera_info'),
        ]
    )
    
    return LaunchDescription([
        grid_size_arg,
        resolution_arg,
        max_height_diff_arg,
        stereo_costmap_node,
    ])