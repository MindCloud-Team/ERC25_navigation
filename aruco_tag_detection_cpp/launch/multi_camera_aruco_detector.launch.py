#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.05',
            description='Size of ArUco markers in meters'
        ),
        
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/lidar/velodyne_points',
            description='LiDAR point cloud topic'
        ),
        
        # ArUco Tag Detector Node
        Node(
            package='aruco_tag_detection_cpp',
            executable='aruco_tag_detector',
            name='multi_camera_aruco_detector',
            output='screen',
            parameters=[{
                'marker_size': LaunchConfiguration('marker_size'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'camera_topics': [
                    '/front_cam/zed_node/rgb/image_rect_color',
                    '/back_cam/zed_node/rgb/image_rect_color', 
                    '/left_cam/zed_node/rgb/image_rect_color',
                    '/right_cam/zed_node/rgb/image_rect_color'
                ]
            }],
            remappings=[
                # You can add remappings here if needed
            ]
        ),
    ])
