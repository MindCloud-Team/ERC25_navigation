#!/usr/bin/env python3
"""
Obstacle layer is a crucial part of the costmap

Detects high obstacles using the readings of LiDAR sensor 

The layer is working with 2D LiDAR for compatibility with the-
-available resources

The layer works simultaneously with other layers and update themselves-
-to be combined to form the costmap

"""

import rclpy
from nav2_costmap_2d.costmap_layer import CostmapLayer
from nav2_costmap_2d import Costmap2D
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.node import Node

class ObstacleLayer(CostmapLayer, Node):
    """
    Class to customize the obstacle layer that inherets from CostmapLayer the-
    -main charachteristics of a layer
    
    
    Attributes:
        Laser_sub (rclpy.subscription.Subscription): Subscription to LaserScan-
        -topic

        obstacle_points (array of float) : store the Cartesian coordinates of 
        obstacles detected by the LiDAR.

        global_frame : Set to be map

    
    ROS Parameters:
        None
    
    ROS Publishers:
        None
    
    ROS Subscribers:
        /scan (sensor_msgs/LaserScan): Subscribes to LiDAR scan data
    
    """
    def __init__(self):
        """
        Initialize the Obstacle layer as both layer and node.
        
        Sets up ROS subscription to LaserScan

        Intialize the obstacle array
        """
        CostmapLayer.__init__(self)
        Node.__init__(self , 'obstacle_layer_node')
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.obstacle_points = []

    def onInitialize(self):
        """
        Called when the layer is initialized

        Sets the global_frame to "map" (the frame for the global costmap)

        Returns:
            bool: True on successful initialization.
        """
        self.global_frame = "map"
        return True
    
    
    def updateBounds(self, min_x, min_y, max_x, max_y):
        """ 
        Update the bounds of the costmap

        Args :
            min_x (float): minimum x-bound
            min_y (float): minimum y-bound
            max_x (float): maximum x-bound
            max_y (float): maximum y-bound
        """
        for x, y in self.obstacle_points:
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
        return (min_x, min_y, max_x, max_y)
    
    def updateCosts(self, costmap):
        """
        Update the costmap with obstacles

        Args:
            costmap (Costmap2D): The costmap to modify.
        """
        for point in self.obstacle_points:
            mx, my = costmap.worldToMap(point[0], point[1])
            if 0 <= mx < costmap.getSizeInCellsX() and 0 <= my < costmap.getSizeInCellsY():
                costmap.setCost(mx, my, self.LETHAL_OBSTACLE)

    def lidar_callback(self, msg):
        """
        Takes the LiDAR readings and process it

        Update obstacle points according to readings
        
        Args:
            msg (LaserScan): Incoming LiDAR scan message.
        """
        self.obstacle_points = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, distance in enumerate(msg.ranges):
            if np.isfinite(distance):  # Ignore infinite values
                angle = angle_min + i * angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                self.obstacle_points.append((x, y))