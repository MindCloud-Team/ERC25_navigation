#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.05',
            description='Size of ArUco markers in meters'
        ),
        
        DeclareLaunchArgument(
            'aruco_dictionary',
            default_value='DICT_4X4_100',
            description='ArUco dictionary to use'
        ),

        # ArUco tag detector node
        Node(
            package='aruco_tag_detection_cpp',
            executable='aruco_tag_detector',
            name='aruco_tag_detector',
            parameters=[{
                'marker_size': LaunchConfiguration('marker_size'),
                'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
                'lidar_topic': '/lidar/velodyne_points',
                'camera_topics': [
                    '/front_cam/zed_node/rgb/image_rect_color',
                    '/back_cam/zed_node/rgb/image_rect_color', 
                    '/left_cam/zed_node/rgb/image_rect_color',
                    '/right_cam/zed_node/rgb/image_rect_color'
                ]
            }],
            output='screen'
        ),
    ])
