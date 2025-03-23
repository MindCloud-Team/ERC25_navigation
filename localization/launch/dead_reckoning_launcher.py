"""
Localization/encoder_imu launch file
Contains 2 nodes:
    Localization using IMU and Encoders
    Localization using Encoders only
Launches both by default, takes argument "run_mode" to launch a specific one
How to launch:
    Imu and Encoders node only:
    ros2 launch localization dead_reckoning_launcher.py run_mode:=imu
    Encoders node only:
    ros2 launch localization dead_reckoning_launcher.py run_mode:=enc
    Both nodes:
    ros2 launch localization dead_reckoning_launcher.py run_mode:=both
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    run_mode = context.launch_configurations['run_mode']

    nodes_to_launch = []

    if run_mode in ['imu', 'both']:
        nodes_to_launch.append(
            Node(
                package='localization',
                executable='dead_reckoning_imu',
                name='localization_imu_encoder',
                output='screen',
                parameters=[{
                    'rate': 50,
                    'wheel_diameter': 0.12,
                    'ticks_per_rev': 600,
                    'base_width': 0.7,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link'
                }]
            )
        )

    if run_mode in ['enc', 'both']:
        nodes_to_launch.append(
            Node(
                package='localization',
                executable='dead_reckoning_enc',
                name='localization_encoder',
                output='screen',
                parameters=[{
                    'rate': 50,
                    'wheel_diameter': 0.12,
                    'ticks_per_rev': 600,
                    'base_width': 0.7,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link'
                }]
            )
        )

    return nodes_to_launch


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'run_mode',
            default_value='both',
            description='Choose which nodes to launch: "imu" for dead_reckoning_imu, "enc" for dead_reckoning_enc, or "both".'
        ),
        OpaqueFunction(function=launch_setup)
    ])
