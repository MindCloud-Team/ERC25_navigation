#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch the custom web UI server."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for the web server'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address for the web server'
    )
    
    # Web UI server node
    web_ui_node = Node(
        package='web_ui',
        executable='web_ui_server',
        name='web_ui_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host')
        }]
    )
    
    return LaunchDescription([
        port_arg,
        host_arg,
        web_ui_node
    ])

