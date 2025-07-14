from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_topics_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('rgb_topics', [])

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    default_config = os.path.join(
        get_package_share_directory('image_compression_util'),
        'config',
        'camera_topics.yaml'
    )

    topics = load_topics_from_yaml(default_config)

    nodes = []
    for topic in topics:
        nodes.append(
            Node(
                package="image_transport",
                executable="republish",
                name=f"compress_{topic.strip('/').replace('/', '_')}",
                # Arguments for the republisher program itself
                arguments=["raw", "compressed"],
                # Remappings handled directly and robustly by the ROS 2 launch system
                remappings=[
                    ('in', topic),
                    ('out', topic)
                ]
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        *nodes
    ])
