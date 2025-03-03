from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization',
            executable='dead_reckoning',
            name='localization_imu_encoder',
            output='screen',
            parameters=[{
                'rate': 50,
                'wheel_diameter': 0.12,
                'ticks_per_rev': 600,
                'base_width': 0.6,
                'frame_id': 'odom',
                'child_frame_id': 'base_link'
            }
            ]
        )
    ])
