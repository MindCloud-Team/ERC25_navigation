#!/usr/bin/env python3

import numpy as np
import cv2
import os
import yaml
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class GlobalCostmap(Node):

    """
    ROS2 node for managing the global costmap.
    
    This node loads a static occupancy grid map from a file, processes it,
    and publishes it as a global costmap.
    
    Attributes:
        map_data (np.ndarray): Processed occupancy grid data.
        map_metadata (dict): Metadata from the YAML file describing the map.
        publisher (rclpy.publisher.Publisher): Publisher for the global costmap.
    
    ROS Parameters:
        N/A (Map file path is currently hardcoded).
    
    ROS Publishers:
        /my_global_costmap (nav_msgs/OccupancyGrid): Publishes the global costmap.
    """

    def __init__(self):

        """
        Initialize the GlobalCostmap node.
        
        Loads the map from a file, sets up the publisher, and processes the map.
        """

        super().__init__('global_costmap')

        # Load map from file
        self.map_data, self.map_metadata = self.load_map_from_file("src/global_costmap/map.yaml")

        if self.map_data is None:
            self.get_logger().error("Failed to load the map.")
            return

        # QoS settings
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher for the costmap
        self.publisher = self.create_publisher(OccupancyGrid, '/my_global_costmap', qos_profile)

        # Process the loaded map
        self.process_map()


    def load_map_from_file(self, yaml_path):

        """
        Load map data from a YAML configuration file and its corresponding image.
        
        Reads the YAML file, extracts metadata, and loads the image as an occupancy grid.
        
        Args:
            yaml_path (str): Path to the YAML file containing map metadata.
        
        Returns:
            tuple: A tuple containing (map_data, map_metadata), where map_data
                   is a NumPy array representation of the occupancy grid.
        """

        try:
            with open(yaml_path, 'r') as file:
                map_metadata = yaml.safe_load(file)

            image_path = map_metadata['image']
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)

            # Load the map image (PGM format)
            image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                self.get_logger().error(f"Failed to load image: {image_path}")
                return None, None

            # Convert grayscale image to occupancy grid (-1 for unknown, 0 for free, 100 for occupied)
            map_data = 100 - (image / 255.0 * 100).astype(np.int8)

            # Add width and height to metadata
            map_metadata['width'] = image.shape[1]
            map_metadata['height'] = image.shape[0]

            return map_data, map_metadata

        except Exception as e:
            self.get_logger().error(f"Error loading map: {str(e)}")
            return None, None


    def process_map(self):

        """
        Process the loaded map and call publish func to be published as an occupancy grid.
        
        Converts the NumPy array representation of the map into a ROS2 OccupancyGrid
        message and calls publish func to publishe it to the /my_global_costmap topic.
        """

        if self.map_data is None or self.map_metadata is None:
            return

        # Get width and height from metadata
        width = self.map_metadata['width']
        height = self.map_metadata['height']

        # Flatten the 2D map to 1D list for ROS message
        self.grid = self.map_data.flatten().tolist()

        # Publish costmap
        # Timer to periodically publish the costmap
        self.timer = self.create_timer(1.0, self.publish_costmap)


    def publish_costmap(self):

        """
        Publishes the generated costmap as an OccupancyGrid message.

        This function checks if map data and metadata are available. If they are,
        it creates an OccupancyGrid message, populates it with relevant information,
        and publishes it.

        The costmap includes:
        - Resolution, width, and height from the map metadata.
        - Origin coordinates from the metadata.
        - The processed map data.
        """

        if self.map_data is None or self.map_metadata is None:
            return

        # Create a new OccupancyGrid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = "map"
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_metadata['resolution']
        costmap_msg.info.width = self.map_metadata['width']
        costmap_msg.info.height = self.map_metadata['height']
        
        # Set the map origin
        origin = list(map(float, self.map_metadata.get('origin', [0.0, 0.0, 0.0])))

        costmap_msg.info.origin = Pose()
        costmap_msg.info.origin.position.x = origin[0]
        costmap_msg.info.origin.position.y = origin[1]
        costmap_msg.info.origin.position.z = 0.0

        # Assign the map data
        costmap_msg.data = self.grid

        # Publish the costmap
        self.publisher.publish(costmap_msg)
        self.get_logger().info('Published global costmap')


def main(args=None):

    """
    Main entry point for the GlobalCostmap node.
    
    Initializes the ROS2 node and starts spinning.
    
    Args:
        args: Command-line arguments.
    """
    
    rclpy.init(args=args)
    global_costmap = GlobalCostmap()
    
    try:
        rclpy.spin(global_costmap)
    except KeyboardInterrupt:
        pass
    finally:
        global_costmap.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()