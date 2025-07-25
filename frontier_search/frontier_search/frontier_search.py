#! /usr/bin/env python3
"""
A frontier search algorithm to choose areas to explore

This algorithm computes costs for each unexplored area and decides which is
best to explore then publishes it as the goal. It groups free cells as
Clusters using Breadth-First Search (BFS).

This corrected version includes state management to prevent goal spamming,
non-blocking server checks, and thread-safe access to shared data.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped  # Used in feedback
from scipy.spatial.transform import Rotation as R
import numpy as np
from math import sqrt, atan2, pi
import threading
from action_msgs.msg import GoalStatus


class FrontierSearch(Node):
    """
    ROS2 Node for choosing next area to explore using frontier search algorithm

    This node subscribes to /map and /odom and sends goals to the
    /navigate_to_pose action server.

    Attributes:
        map_sub (Subscriber): Subscriber to 'Occupancy grid'
        odom_sub (Subscriber): Subscriber to odom
        goal_client (ActionClient): Client for the NavigateToPose action server
        rover_position (tuple): Rover's current position (x, y)
        yaw (float, radians): Rover's current yaw
        last_goal (tuple): Last computed goal (world coordinates)
        origin (Pose): Occupancy grid's origin
        resolution (float): Occupancy grid's resolution
        frontier_cells (list): List of all frontier cells to explore
        is_navigating (bool): State flag, True if the robot is currently executing a goal
        lock (threading.Lock): Lock to ensure thread-safe access to shared variables
    """

    def __init__(self):
        """
        Initialize the FrontierSearch Node.
        Sets up ROS publishers, subscribers, and state variables.
        """
        super().__init__("frontier_search_node")

        # Declare and get parameters
        self.declare_parameter("a", 1.0)  # Alpha for Distance
        self.declare_parameter("b", 10.0)  # Beta for Size
        self.declare_parameter("g", 0.5)  # Gamma for Orientation
        self.a = self.get_parameter("a").value
        self.b = self.get_parameter("b").value
        self.g = self.get_parameter("g").value

        # --- MODIFIED: State Management & Threading ---
        self.is_navigating = False
        self.lock = threading.Lock()
        self.rover_position = (0.0, 0.0)
        self.yaw = 0.0
        self.last_goal = None
        self.origin = None
        self.resolution = 0.0
        self.frontier_cells = []

        # Create Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.mapCallback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.get_current_position, 10
        )

        # Create action client
        self.goal_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # --- MODIFIED: Wait for action server in __init__ to avoid blocking callbacks ---
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self.goal_client.wait_for_server()
        self.get_logger().info("Action server is available.")

    def mapCallback(self, map_msg: OccupancyGrid):
        """
        Main function triggered by map updates. Controls the exploration logic.
        """
        # --- MODIFIED: State check to prevent re-calculating while navigating ---
        if self.is_navigating:
            self.get_logger().info(
                "Robot is navigating, skipping new frontier search.",
                throttle_duration_sec=10,
            )
            return

        self.get_logger().info("Received new map. Searching for frontiers...")

        # Store current grid's data
        width = map_msg.info.width
        height = map_msg.info.height
        self.origin = map_msg.info.origin
        self.resolution = map_msg.info.resolution
        data = np.array(map_msg.data).reshape((height, width))

        # Find all frontier cells (free cells adjacent to unknown cells)
        self.frontier_cells = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # A frontier cell must be a known free cell (value 0)
                if data[y, x] == 0:
                    # Check its 8 neighbors
                    neighbours = data[y - 1 : y + 2, x - 1 : x + 2].flatten()
                    # If any neighbor is an unknown cell (value -1), it's a frontier
                    if -1 in neighbours:
                        self.frontier_cells.append((x, y))

        if not self.frontier_cells:
            self.get_logger().warn(
                "No frontier cells found. Exploration may be complete."
            )
            return

        self.get_logger().info(f"Found {len(self.frontier_cells)} frontier cells.")

        # Group frontier cells into clusters
        clusters = self.cluster_cells()
        if not clusters:
            self.get_logger().warn("Could not form any frontier clusters.")
            return

        self.get_logger().info(f"Grouped into {len(clusters)} frontier clusters.")

        # Find the best cluster to visit based on a cost function
        best_cost = float("inf")
        best_centroid = None
        for cluster in clusters:
            # Filter out very small clusters that might just be noise
            if len(cluster) < 5:
                continue
            centroid = self.compute_centroid(cluster)
            cost = self.compute_cost(centroid, len(cluster))
            if cost < best_cost:
                best_cost = cost
                best_centroid = centroid

        # If a valid best frontier is found, send it as the next goal
        if best_centroid:
            cx_grid, cy_grid = best_centroid
            world_x = cx_grid * self.resolution + self.origin.position.x
            world_y = cy_grid * self.resolution + self.origin.position.y
            current_goal = (world_x, world_y)

            if current_goal != self.last_goal:
                self.last_goal = current_goal
                self.request_goal(world_x, world_y)
            else:
                self.get_logger().info(
                    "Best frontier is the same as the last goal. Waiting for map to change."
                )
        else:
            self.get_logger().info("No suitable frontier found in this iteration.")

    def cluster_cells(self):
        """
        Groups adjacent frontier cells into clusters using a Breadth-First Search approach.
        """
        clusters = []
        visited = set()
        frontier_set = set(self.frontier_cells)

        for cell in self.frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                current = queue.pop(0)
                cluster.append(current)
                x, y = current

                # Check 8 adjacent neighbors
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (x + dx, y + dy)
                        if neighbor in frontier_set and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)

            clusters.append(cluster)
        return clusters

    def compute_centroid(self, cluster):
        """
        Computes the centroid (average position) of a cluster of cells.
        """
        x_vals = [pt[0] for pt in cluster]
        y_vals = [pt[1] for pt in cluster]
        return (sum(x_vals) / len(x_vals), sum(y_vals) / len(y_vals))

    def compute_cost(self, centroid_grid, cluster_size):
        """
        Computes the cost for a frontier cluster.
        Cost = a*(distance to robot) - b*(cluster size) + g*(orientation penalty)
        Note: The original formula used +b/cluster_size, which favors smaller clusters.
              Changed to -b*cluster_size to favor larger, more promising frontiers.
        """
        # --- MODIFIED: Use lock for thread-safe read of shared variables ---
        with self.lock:
            rx, ry = self.rover_position
            current_yaw = self.yaw

        # Convert centroid from grid coordinates to world coordinates
        cx_grid, cy_grid = centroid_grid
        world_x = cx_grid * self.resolution + self.origin.position.x
        world_y = cy_grid * self.resolution + self.origin.position.y

        # Cost component 1: Distance from robot to frontier
        dx = world_x - rx
        dy = world_y - ry
        distance = sqrt(dx**2 + dy**2)

        # Cost component 2: Orientation penalty
        angle_to_goal = atan2(dy, dx)
        heading_diff = abs(current_yaw - angle_to_goal)
        # Normalize angle difference to be within [0, pi]
        heading_diff = min(heading_diff, 2 * pi - heading_diff)
        orientation_penalty = heading_diff / pi  # Normalize to [0, 1]

        # Final Cost Function: Lower is better
        # We want to minimize distance and orientation change, and maximize size.
        cost = (
            (self.a * distance)
            - (self.b * cluster_size)
            + (self.g * orientation_penalty)
        )
        return cost

    def request_goal(self, world_x, world_y):
        """
        Sends the calculated goal to the navigation action server.
        """
        if not self.goal_client.server_is_ready():
            self.get_logger().error("Action server is not available. Cannot send goal.")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = world_x
        goal.pose.pose.position.y = world_y
        goal.pose.pose.orientation.w = (
            1.0  # No specific orientation, Nav2 will handle it
        )

        self.get_logger().info(f"Sending goal: ({world_x:.2f}, {world_y:.2f})")

        goal_future = self.goal_client.send_goal_async(
            goal=goal, feedback_callback=self.goal_feedback_callback
        )
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback triggered when the server accepts or rejects the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by action server.")
            return

        # --- MODIFIED: Set navigating state to True ---
        self.is_navigating = True
        self.get_logger().info("Goal accepted. Robot is navigating...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """
        Callback triggered when the navigation action finishes.
        """
        # The result object contains the final status and the result message
        action_result = future.result()
        status = action_result.status
        result_message = (
            action_result.result
        )  # This is the nav2_msgs/NavigateToPose.Result

        # Check the overall action status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Now, check the specific error_code from the Nav2 result
            if result_message.error_code == 0:  # 0 means NONE/SUCCESS
                self.get_logger().info("Goal succeeded!")
            else:
                self.get_logger().warn(
                    f"Goal succeeded with error code {result_message.error_code}: "
                    f"{result_message.error_msg}"
                )
        else:
            # The action itself failed (e.g., was aborted, canceled)
            self.get_logger().error(f"Goal failed with status: {status}")

        # Reset state so the node can search for a new goal
        self.is_navigating = False
        self.last_goal = None  # Clear last goal to allow retrying if it failed

    def goal_feedback_callback(self, feedback_msg):
        """
        Receives and logs feedback from the navigation action server.
        """
        feedback = feedback_msg.feedback
        # Log feedback occasionally to avoid spamming the console
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} m",
            throttle_duration_sec=5,
        )

    def get_current_position(self, msg: Odometry):
        """
        Updates the robot's current position and orientation from /odom.
        """
        # --- MODIFIED: Use lock for thread-safe write of shared variables ---
        with self.lock:
            self.rover_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            orientation = msg.pose.pose.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            # Use as_euler to get yaw, which is rotation around z-axis
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
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
