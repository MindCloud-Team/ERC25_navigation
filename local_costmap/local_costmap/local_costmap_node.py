#!/usr/bin/env python3
"""
Local costmap generation module for rover navigation.

This module processes LIDAR data to create a local costmap around the rover,
which is used for obstacle avoidance and path planning.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import math

class LocalCostmapNode(Node):

    """
    ROS2 node for generating a local costmap using LIDAR data.

    This node subscribes to LIDAR scan data, processes it to create a 2D occupancy grid,
    and publishes the costmap for use by the planning layer.

    Attributes:
        _grid_size (float): Size of the costmap in meters (width and height).
        _resolution (float): Resolution of the costmap in meters per cell.
        _range_limit (float): Maximum range of LIDAR data to consider.
        _grid_width (int): Width of the costmap in cells.
        _grid_height (int): Height of the costmap in cells.
        _scan_sub (rclpy.subscription.Subscription): Subscription to LIDAR scan topic.
        _costmap_pub (rclpy.publisher.Publisher): Publisher for the local costmap.
        _timer (rclpy.timer.Timer): Timer for periodic costmap publishing.
        _lidar_data (sensor_msgs.msg.LaserScan): Latest LIDAR scan data.

    ROS Parameters:
        ~grid_size (float): Size of the costmap in meters. Defaults to 4.0.
        ~resolution (float): Resolution of the costmap in meters per cell. Defaults to 0.05.
        ~range_limit (float): Maximum range of LIDAR data to consider. Defaults to 2.0.

    ROS Publishers:
        /local_costmap (nav_msgs/OccupancyGrid): Publishes the local costmap.

    ROS Subscribers:
        /scan (sensor_msgs/LaserScan): Subscribes to LIDAR scan data.
    """

    def __init__(self):
        
        """
        Initialize the LocalCostmapNode and set up ROS publishers, subscribers, and parameters.
        """
        
        super().__init__('local_costmap')

        self.grid_size = 4.0  # 4x4 meters grid (centered on robot)
        self.resolution = 0.05  # Each cell is 5 cm
        self.range_limit = 2.0  # 2 meters max for LIDAR range

        self.grid_width = int(self.grid_size / self.resolution)
        self.grid_height = int(self.grid_size / self.resolution)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)

        self.timer = self.create_timer(0.1, self.publish_costmap)

        self.lidar_data = None

    
    def scan_callback(self, msg):
        
        """
        Process incoming LIDAR scan messages and store the data.

        Args:
            msg (sensor_msgs.msg.LaserScan): The LIDAR scan message.
        """
        
        self.lidar_data = msg


    def create_costmap(self):
        
        """
        Convert LIDAR scan data to an occupancy grid.

        Returns:
            numpy.ndarray: A 2D grid representing the local costmap.
        """

        if self.lidar_data is None:
            self.get_logger().warn("No LIDAR data received yet.")
            return None

        # Initialize empty costmap (-1: Unknown)
        grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        angle = self.lidar_data.angle_min
        
        for r in self.lidar_data.ranges:
            if r < self.range_limit and r > self.lidar_data.range_min:
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                gx = int((x + self.grid_size / 2) / self.resolution)
                gy = int((y + self.grid_size / 2) / self.resolution)

                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    grid[gy, gx] = 100  # Mark as occupied

            angle += self.lidar_data.angle_increment

        self.add_inflation(grid)
        return grid

    def add_inflation(self, grid):
        """
        Apply an inflation layer to the costmap to account for obstacle proximity.

        Args:
            grid (numpy.ndarray): The occupancy grid where obstacles are inflated.
        """
        inflation_radius = int(0.3 / self.resolution)  # 30 cm buffer converted to grid cells
        max_cost = 100
        decay_rate = 100 / inflation_radius  # Linear decay

        # Create a copy of the grid to apply inflation
        inflated_grid = np.copy(grid)

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if grid[y, x] == 100:  # Found an obstacle
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                dist = math.sqrt(dx**2 + dy**2)
                                if dist <= inflation_radius:
                                    new_cost = max_cost - int(decay_rate * dist)
                                    inflated_grid[ny, nx] = max(inflated_grid[ny, nx], new_cost)
        
        np.copyto(grid, inflated_grid)

    def publish_costmap(self):
        
        """
         Publish the local costmap as an OccupancyGrid. 
        """
        
        grid = self.create_costmap()
        if grid is None:
            return

        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "laser"  # Local costmap relative to robot

        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.grid_width
        costmap_msg.info.height = self.grid_height

        costmap_msg.info.origin = Pose()
        costmap_msg.info.origin.position.x = -self.grid_size / 2
        costmap_msg.info.origin.position.y = -self.grid_size / 2
        costmap_msg.info.origin.position.z = 0.0

        costmap_msg.data = grid.flatten().tolist()

        self.costmap_pub.publish(costmap_msg)
        self.get_logger().info("Published local costmap")


def main(args=None):
    
    """
    Main entry point for the local costmap node.

    Args:
        args: Command-line arguments.
    """
    
    rclpy.init(args=args)
    node = LocalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()