from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='microcontroller',
            executable='microcontroller_interface',
            name='microcontroller_interface',
            output='screen',
            parameters=[{
                'microcontroller_list': ["hi", "bye"],
                'topics_list': ["/cmd_vel:geometry_msgs/Twist"],
                'publish_topics_list': ["/right_ticks:std_msgs/Int16", "/left_ticks:std_msgs/Int16"],
                'microcontroller_ports': ["ttyACM0"]
            }]
        )
    ])
