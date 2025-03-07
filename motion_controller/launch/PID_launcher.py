from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_controller',
            executable='PID_controller',
            name='motion_control',
            output='screen',
            parameters=[{
                'rate': 50,
                'linear_kp': 1.0,
                'linear_ki': 0.5,
                'linear_kd': 0.2,
                'angular_kp': 1.2,
                'angular_ki': 0.3,
                'angular_kd': 0.4
            }
            ]
        )
    ])