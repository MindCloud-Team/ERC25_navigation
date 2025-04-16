#!/usr/bin/env python3
"""
Global Planner Node for Pathfinding

This module implements the A* algorithm for global path planning in a 2D grid environment.
It receives odometry data to determine the current position and computes an optimal path
from the start position to a goal position while avoiding obstacles.

System Architecture: Planning Layer

ROS2 Topics:
    - Publishers:
        - Topic: /global_planner 
          Type: nav_msgs/Path
          Purpose: Publishes the computed global path.
    - Subscribers:
        - Topic: /odom
          Type: nav_msgs/Odometry
          Purpose: Receives the robot's current position from odometry data.
        - Topic: /map
          Type: nav_msgs/OccupancyGrid
          Purpose: Receives the map and the costmap of the rover's environment.

Algorithm:
    - Uses A* (A-star) algorithm to find the shortest path from the start to the goal.
    - Incorporates obstacle avoidance using a SLAM-based costmap.
    - Supports diagonal movement for a more natural path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
import heapq

class GlobalPlannerNode(Node):
    """
    Global Planner Node for computing optimal paths using A* algorithm.

    Attributes:
        path_publisher               : Publishes the computed path.
        odom_subscriber              : Subscribes to odometry data.
        map_subscriber               : Subscribes to SLAM occupancy grid
        map_width                    : Map width.
        map_height                   : Map height.
        map_resolution               : Map resolution.
        map_origin_x , map_origin_y  : Map orgin as x , y.
        current_x , current_y        : Rover's current position from odom.
        goal_x , goal_y              : Goal position as x , y in meters.    
        orthogonal_step_cost         : Cost for orthogonal movement.
        diagonal_step_cost           : Cost for diagonal movement.
    """

    def __init__(self):
        super().__init__('global_planner')
        self.get_logger().info("Global Planner Node Initialized")

        qos_prof = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # ROS 2 Publishers & Subscribers
        self.path_publisher = self.create_publisher(Path, 'global_planner', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_prof)

        # Map Information (Initialized as None)
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.current_x = None
        self.current_y = None
        self.goal_x, self.goal_y = 1, -3

        # Path Planning Parameters
        self.orthogonal_step_cost = 1.0
        self.diagonal_step_cost = self.orthogonal_step_cost * 1.41421

    def map_callback(self, msg):
        """ 
        Receives the SLAM-generated map 

        Args:
            msg(nav_msgs.msg.OccupancyGrid): Received map.
        """
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.get_logger().info("Map received and stored.")

    def odom_callback(self, msg):
        """
        Process odometry data and update the current position.

        Args:
            msg (nav_msgs.msg.Odometry): Received odometry message.
        """
        if self.map_data is None:
            self.get_logger().warn("Map not received yet. Skipping path computation.")
            return

        # Convert odometry position from meters to grid coordinates
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.current_x, self.current_y = self.meters_to_grid(msg.pose.pose.position.x, msg.pose.pose.position.y)
        goal_x, goal_y = self.meters_to_grid(self.goal_x, self.goal_y)
        # Compute global path
        path = self.A_Algorithm(self.current_x, self.current_y, goal_x, goal_y)
        if path:
            self.publish_path(path)
            self.get_logger().info("path published!")
            # self.get_logger().info(f"path:{path}")
        else:
            self.get_logger().warn("No valid path found!")

    def meters_to_grid(self, x_meters, y_meters):
        """
        Converts the x , y coordinates from real_world scale to grid scale
        Args:
            y_meters : The real_world y coordinate.
            x_meters : The real_world x coordinate.

        """
        if self.map_origin_x is not None and self.map_resolution is not None:
            x_grid = int((x_meters - self.map_origin_x) // self.map_resolution)
            y_grid = int((y_meters - self.map_origin_y) // self.map_resolution)

            return x_grid, y_grid
        else:
            self.get_logger().warn("resolurion and map origin isn't recieved yet!")
            return

    def grid_to_meter(self, x_grid, y_grid):
        """
        Converts the x , y coordinates from grid scale to real_world scale
        Args:
            y_grid : The grid y coordinate.
            x_grid : The grid x coordinate.

        """
        if self.map_origin_x is not None and self.map_resolution is not None:
            x_meter = (self.map_resolution * x_grid) + self.map_origin_x
            y_meter = (self.map_resolution * y_grid) + self.map_origin_y
            return x_meter, y_meter
        else:
            self.get_logger().warn("resolurion and map origin isn't recieved yet!")
            return

    def h(self, x, y, goal_x, goal_y):
        """
        Compute the heuristic distance between two points.

        Args:
            x , y           : Current position.
            goal_x , goal_y : Goal position.

        Returns:
            float: Euclidean distance between current and goal.
        """
        return np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)

    def valid_nodes(self, current):
        """
        Identifies valid neighboring nodes for movement, considering obstacle costs.

        Args:
            current : The (x, y) coordinates of the current position.

        Returns:
            list: A list of tuples representing valid neighboring nodes and their associated movement costs.
        """
        neighbors = []
        x, y = current
        LETHAL_COST = 50
        moves = [
            (1, 0, self.orthogonal_step_cost),    # Right
            (-1, 0, self.orthogonal_step_cost),   # Left
            (0, -1, self.orthogonal_step_cost),   # Down
            (0, 1, self.orthogonal_step_cost),    # Up
            (-1, -1, self.diagonal_step_cost),    # Lower-left
            (-1, 1, self.diagonal_step_cost),     # Upper-left
            (1, -1, self.diagonal_step_cost),     # Lower-right
            (1, 1, self.diagonal_step_cost)       # Upper-right
        ]

        for dx, dy, step_cost in moves:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.map_data[ny, nx] is not None and self.map_data[ny, nx] < LETHAL_COST:  # Threshold for obstacles
                    adjusted_cost = step_cost + self.map_data[ny, nx] / 255  # New step cost
                    neighbors.append(((nx, ny), adjusted_cost))
        return neighbors

    def A_Algorithm(self, start_x, start_y, goal_x, goal_y):
        """
        Implements the A* Algorithm for pathfinding.

        Args:
            start_x , start_y : (x, y) coordinates of the start position.
            goal_x , goal_y   : (x, y) coordinates of the goal position.

        Returns:
            list: The computed path as a list of (x, y) coordinates.
        """
        self.get_logger().info(f"Starting A* from {start_x, start_y} to {goal_x, goal_y}")

        open_set = []
        heapq.heappush(open_set, (0, (start_x, start_y)))
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.h(start_x, start_y, goal_x, goal_y)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (goal_x, goal_y):
                return self.reconstruct_path(came_from, current)

            for neighbor, cost in self.valid_nodes(current):
                temp_g_score = g_score[current] + cost
                if neighbor not in g_score or temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    x_new, y_new = neighbor
                    f_score[neighbor] = temp_g_score + self.h(x_new, y_new, goal_x, goal_y)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from goal to start using backtracking.

        Args:
            came_from : Dictionary mapping each node to its predecessor.
            current   : Index of the goal node.

        Returns:
            path      : Reconstructed path.
        """
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()

        return path

    def publish_path(self, path):
        """
        Publishes the computed path to the /global_planner topic.

        Args:
            path : The computed path as a list of (x, y) coordinates.
        """
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for x, y in path:
            pose = PoseStamped()
            x, y = self.grid_to_meter(x, y)
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            msg.poses.append(pose)

        self.path_publisher.publish(msg)


def main(args=None):
    """
    Main entry point for the global planner node.

    Args:
        args: Command-line arguments
    """
    rclpy.init(args=args)
    planner = GlobalPlannerNode()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
