from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_controller',
            executable='PID_controller',
            name='motion_control_our',
            output='screen',
            parameters=[{
                'rate': 50,
                'linear_kp': 2.0,
                'linear_ki': 0.1,
                'linear_kd': 0.3,
                'angular_kp': 2.0,
                'angular_ki': 0.1,
                'angular_kd': 0.3
            }
            ]
        )
    ])
