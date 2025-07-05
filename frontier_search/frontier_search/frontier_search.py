#! /usr/bin/env python3
"""
A frontier search algorithm to choose areas to explore

This algorithm computes costs for each unexplored area and decides which is
best to explore then publishes it as the goal
It groups free cells as Clusters using Breadth-First Search (BFS)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from math import sqrt, abs, min, atan2, pi


class FrontierSearch(Node):
    """
    ROS2 Node for choosing next area to explore using frontier search algorithm

    This node subscribes to /my_global_costmap and /odom and publishes to
    /search_goal topics

    Attributes:
        map_sub (Subscriber): Subscriber to 'Occupancy grid'
        odom_sub (Subscriber): Subscriber to odom
        goal_pub (Publisher): Publishes the goal to search_goal
        rover_position (tuple): Rover's current position
        yaw (float, radians): Rover's current yaw
        last_goal (tuple): Last Computed goal
        origin (Pose): Occupancy grid's origin
        resolution (float): Occupancy grid's resolution
        frontier_cells (list): list of all frontier cells to explore

    ROS Parameters:
        a (float): Alpha for Distance
        b (float): Beta for Size
        g (float): Gamma for Orientation

    ROS Subscriber:
        /my_global_costmap (nav_msgs/OccupancyGrid)
        /odom (nav_msgs/Odometry)

    ROS Publisher:
        /search_goal (geometry_msgs/PoseStamped)

    """
    def __init__(self):
        """
        Initialize the FrontierSearch Node.

        Sets up ROS publishers, subscribers.
        """
        super().__init__("frontier_search_node")

        # Declare Parameters
        self.declare_parameter("a", 1.0) # Alpha for Distance
        self.declare_parameter("b", 10.0) # Beta for Size
        self.declare_parameter("g", 0.5) # Gamma for Orientation

        # Get Parameters
        self.a = self.get_parameter("a").value
        self.b = self.get_parameter("b").value
        self.g = self.get_parameter("g").value

        # Set Variables
        self.rover_position = (0.0, 0.0)
        self.yaw = 0.0
        self.last_goal = None

        # Create Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/my_global_costmap",
            self.mapCallback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.get_current_position,
            10
        )
        
        # Create Publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/search_goal",
            10
        )

    def mapCallback(self, map:OccupancyGrid):
        """
        Main Function which receives the occupancy grid and control the flow
        of the code
        """
        # Store current grid's data
        width = map.info.width
        height = map.info.height
        self.origin = map.info.origin
        self.resolution = map.info.resolution
        data = np.array(map.data).reshape((height, width))

        # List of all frontier cells to explore
        self.frontier_cells = []

        for y in range(1, height-1):
            for x in range(1, width - 1):
                if data[y,x] == 0:
                    neighbours = data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbours:
                        self.frontier_cells.append((x,y))
        
        self.get_logger().info(f"Found {len(self.frontier_cells)} frontier cells")

        # Group cells into Clusters
        clusters = self.cluster_cells()
        self.get_logger().info(f"Found {len(clusters)} frontier clusters")
        
        # Loop through all created clusters and find the best one to visit
        best_cost = float('inf')
        best_centroid = None

        for cluster in clusters:
            centroid = self.compute_centroid(cluster)
            cost = self.compute_cost(centroid, len(cluster))
            if cost < best_cost:
                best_cost = cost
                best_centroid = centroid
        
        # Publish to goal if best_centroid is found
        if best_centroid:
            cx, cy = best_centroid
            world_x = cx * self.resolution + self.origin.position.x
            world_y = cy * self.resolution + self.origin.position.y
            current_goal = (world_x, world_y)
            if current_goal != self.last_goal:
                self.last_goal = current_goal
                self.publish_goal(world_x, world_y)
            else:
                self.get_logger().info("Same goal as last time. Not publishing.")

    def cluster_cells(self):
        """
        Creates clusters of cells
        """
        clusters = [] # List of all clusters to be returned
        visited = set() # A set of already visited cells
        frontier_set = set(self.frontier_cells)

        for cell in self.frontier_cells:
            if cell in visited:
                continue

            queue = [cell] # Cells to explore
            cluster = [] # Current Cluster

            # Loop through the queue until finished to form a cluster
            while queue:
                current = queue.pop(0)
                if current in visited:
                    continue
                visited.add(current)
                cluster.append(current)

                x, y = current
                neighbours = [(x+dx, y+dy)
                              for dx in [-1, 0, 1]
                              for dy in [-1, 0, 1]
                              if not (dx == 0 and dy == 0)] # 8 Adjacent cells (BFS Expansion)
                
                for nx, ny in neighbours:
                    neighbour = (nx, ny)
                    if neighbour in frontier_set and neighbour not in visited:
                        queue.append(neighbour)

            clusters.append(cluster)

        return clusters
    
    def compute_centroid(self, cluster):
        """
        Computes centroids of clusters to later find the distance between
        rover and clusters

        Args:
            cluster (list)
        """
        x_vals = [pt[0] for pt in cluster]
        y_vals = [pt[1] for pt in cluster]
        return (sum(x_vals) / len(x_vals), sum(y_vals) / len(y_vals))
    
    def compute_cost(self, centroid, cluster_size):
        """
        Computes cost for each cluster to find the best cost and decides to
        explore it

        Args:
            centroid (tuple)
            cluster_size (int)
        """
        # Distance:
        rx, ry = self.rover_position
        cx, cy = centroid

        world_x = cx * self.resolution + self.origin.position.x
        world_y = cy * self.resolution + self.origin.position.y

        dx = world_x - rx
        dy = world_y - ry
        distance = sqrt(dx**2 + dy**2)

        # Orientation:
        angle_to_goal = atan2(dy, dx)
        heading_diff = abs(self.yaw - angle_to_goal)
        heading_diff = min(heading_diff, 2 * pi - heading_diff)
        orientation_penalty = heading_diff / pi  # Normalize to [0, 1]

        # Cost Function
        return self.a*distance + self.b/cluster_size + self.g*orientation_penalty

    def publish_goal(self, world_x, world_y):
        """
        Publishes the goal
        
        Args:
            world_x (float)
            world_y (float)
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = world_x
        goal.pose.position.y = world_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published goal at ({world_x:.2f}, {world_y:.2f}) ")
    
    def get_current_position(self, msg: Odometry):
        """
        Stores rover's current position and orientation

        Args:
            msg (nav_msgs/Odometry)
        """
        self.rover_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = R.from_quat(q).as_euler('xyz', degrees=False)

def main(args=None):
    """
    Main entry point for the frontier search node.
    """
    rclpy.init(args=args)
    node = FrontierSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()