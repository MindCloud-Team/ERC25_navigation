#! /usr/bin/env python3
"""
A frontier search algorithm to choose areas to explore

This algorithm computes costs for each unexplored area and decides which is
best to explore then publishes it as the goal
It groups free cells as Clusters using Breadth-First Search (BFS)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
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
        goal_client (ActionClient): Request the goal send to search_goal server
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

    ROS Action Client:
        /search_goal (nav2_msgs/NavigateToPose)

    """

    def __init__(self):
        """
        Initialize the FrontierSearch Node.

        Sets up ROS publishers, subscribers.
        """
        super().__init__("frontier_search_node")

        # Declare Parameters
        self.declare_parameter("a", 1.0)  # Alpha for Distance
        self.declare_parameter("b", 10.0)  # Beta for Size
        self.declare_parameter("g", 0.5)  # Gamma for Orientation

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
            OccupancyGrid, "/my_global_costmap", self.mapCallback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.get_current_position, 10
        )

        # Create action client
        self.goal_client = ActionClient(self, NavigateToPose, "/search_goal")

    def mapCallback(self, map: OccupancyGrid):
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

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:
                    neighbours = data[y - 1 : y + 2, x - 1 : x + 2].flatten()
                    if -1 in neighbours:
                        self.frontier_cells.append((x, y))

        self.get_logger().info(f"Found {len(self.frontier_cells)} frontier cells")

        # Group cells into Clusters
        clusters = self.cluster_cells()
        self.get_logger().info(f"Found {len(clusters)} frontier clusters")

        # Loop through all created clusters and find the best one to visit
        best_cost = float("inf")
        best_centroid = None

        for cluster in clusters:
            if self.is_cluster_trapped(cluster, data, width, height):
                continue  # Skip trapped clusters

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
                self.request_goal(world_x, world_y)
            else:
                self.get_logger().info("Same goal as last time. Not publishing.")

    def cluster_cells(self):
        """
        Creates clusters of cells
        """
        clusters = []  # List of all clusters to be returned
        visited = set()  # A set of already visited cells
        frontier_set = set(self.frontier_cells)

        for cell in self.frontier_cells:
            if cell in visited:
                continue

            queue = [cell]  # Cells to explore
            cluster = []  # Current Cluster

            # Loop through the queue until finished to form a cluster
            while queue:
                current = queue.pop(0)
                if current in visited:
                    continue
                visited.add(current)
                cluster.append(current)

                x, y = current
                neighbours = [
                    (x + dx, y + dy)
                    for dx in [-1, 0, 1]
                    for dy in [-1, 0, 1]
                    if not (dx == 0 and dy == 0)
                ]  # 8 Adjacent cells (BFS Expansion)

                for nx, ny in neighbours:
                    neighbour = (nx, ny)
                    if neighbour in frontier_set and neighbour not in visited:
                        queue.append(neighbour)

            clusters.append(cluster)

        return clusters

    def is_cluster_trapped(
        self, cluster, data, width, height, steps=5, obstacle_thresh=0.7
    ):
        """
        Checks if a frontier cluster is likely enclosed by obstacles.

        Args:
            cluster (list): List of (x, y) grid cells in the cluster.
            data (np.ndarray): 2D occupancy grid.
            width (int): Map width.
            height (int): Map height.
            steps (int): Max layers to expand.
            obstacle_thresh (float): Threshold ratio to consider trapped.

        Returns:
            bool: True if trapped, False if reachable.
        """
        # Not revised yet
        # Convert to set for fast lookup
        cluster_set = set(cluster)

        for step in range(1, steps + 1):
            ring_cells = set()

            for x, y in cluster:
                for dx in range(-step, step + 1):
                    for dy in [-step, step]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            ring_cells.add((nx, ny))
                    for dy in range(-step + 1, step):
                        for dx in [-step, step]:
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                ring_cells.add((nx, ny))

            # Filter out cells already in cluster
            ring_cells -= cluster_set

            if not ring_cells:
                continue

            obstacle_count = sum(1 for (x, y) in ring_cells if data[y, x] == 100)
            ratio = obstacle_count / len(ring_cells)

            if ratio > obstacle_thresh:
                self.get_logger().info(
                    f"Cluster discarded at step {step} with {ratio*100:.1f}% obstacles"
                )
                return True  # Trapped

        return False  # Reachable

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
        return self.a * distance + self.b / cluster_size + self.g * orientation_penalty

    def request_goal(self, world_x, world_y):
        """
        Sends the goal to the action server

        Args:
            world_x (float)
            world_y (float)
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = world_x
        goal.pose.pose.position.y = world_y
        goal.pose.pose.orientation.w = 1.0
        self.goal_client.wait_for_server()
        goal_future = self.goal_client.send_goal_async(
            goal=goal, feedback_callback=self.goal_client_feedback
        )
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called once the goal is accepted or rejected
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by action server.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """
        Called when the action result is received
        """
        result = future.result().result
        self.get_logger().info(
            f"Goal completed with status {result.error_code}: {result.error_msg}"
        )

    def goal_client_feedback(self, feedback_msg: NavigateToPose_FeedbackMessage):
        """
        Receives feedback from action server and logs them
        """
        self.get_logger().info(
            f"Feedback:: Current X: \
                               {feedback_msg.feedback.current_pose.pose.x},\
                                Current Y: {feedback_msg.feedback.current_pose.y}"
        )

    def get_current_position(self, msg: Odometry):
        """
        Stores rover's current position and orientation

        Args:
            msg (nav_msgs/Odometry)
        """
        self.rover_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = R.from_quat(q).as_euler("xyz", degrees=False)


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


if __name__ == "__main__":
    main()
