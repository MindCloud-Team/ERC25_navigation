#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import math
import struct


class StereoCostmapNode(Node):
    def __init__(self):
        super().__init__("stereo_costmap")

        # Parameters
        self.grid_size = 6.0  # 6x6 meters grid
        self.resolution = 0.1  # 10 cm per cell
        self.max_height_diff = 0.3  # Max height difference for traversable terrain
        self.min_obstacle_height = 0.15  # Min height to consider as obstacle
        self.robot_height = 0.2  # Approximate robot height
        self.range_limit = 5.0  # Maximum range to consider

        self.grid_width = int(self.grid_size / self.resolution)
        self.grid_height = int(self.grid_size / self.resolution)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Stereo camera parameters (will be updated from camera info)
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.R = None  # Rotation matrix
        self.T = None  # Translation vector
        self.baseline = None
        self.focal_length = None

        # Image storage - Updated for ZED camera
        self.rgb_image = None
        self.depth_image = None
        self.pointcloud_data = None
        self.camera_info = None

        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

        # Subscribers - Updated for ZED camera
        self.rgb_image_sub = self.create_subscription(
            Image,
            "/front_cam/zed_node/rgb/image_rect_color",
            self.rgb_image_callback,
            10,
        )
        self.depth_image_sub = self.create_subscription(
            Image, "/front_cam/zed_node/depth", self.depth_image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            "/lidar/velodyne_points",
            self.pointcloud_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/front_cam/zed_node/rgb/camera_info",
            self.camera_info_callback,
            10,
        )

        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/stereo_costmap", 10)
        # self.pointcloud_pub = self.create_publisher(
        #     PointCloud2, "/lidar/velodyne_points", 10
        # )

        # Timer for processing - reduced frequency for better performance
        self.timer = self.create_timer(0.5, self.process_zed_data)

        self.get_logger().info("ZED costmap node initialized")

    def rgb_image_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting RGB image: {e}")

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def pointcloud_callback(self, msg):
        self.pointcloud_data = msg

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_matrix_left = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs_left = np.array(msg.d)
        self.focal_length = msg.k[0]  # fx

    def extract_points_from_pointcloud(self):
        if self.pointcloud_data is None:
            return None

        points_3d = []

        # Parse PointCloud2 data with optimized sampling
        point_step = self.pointcloud_data.point_step
        data = self.pointcloud_data.data

        # Sample every 8th point for performance (reduces from 400k to ~50k points)
        sample_step = point_step * 8

        # Assuming XYZ format (12 bytes per point)
        for i in range(0, len(data), sample_step):
            if i + 12 <= len(data):
                x, y, z = struct.unpack("fff", data[i : i + 12])

                # Filter out invalid points and points outside range
                if (
                    not (np.isnan(x) or np.isnan(y) or np.isnan(z))
                    and 0.1 < z < self.range_limit
                ):
                    points_3d.append([x, y, z])

        return np.array(points_3d) if points_3d else None

    def extract_points_from_depth(self):
        if self.depth_image is None or self.camera_info is None:
            return None

        points_3d = []
        height, width = self.depth_image.shape

        # Camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Convert depth image to 3D points with aggressive sampling
        for v in range(0, height, 8):  # Sample every 8th pixel for better performance
            for u in range(0, width, 8):
                depth = self.depth_image[v, u]

                # Filter valid depth values
                if not np.isnan(depth) and 0.1 < depth < self.range_limit:
                    # Convert pixel coordinates to 3D point
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth

                    points_3d.append([x, y, z])

        return np.array(points_3d) if points_3d else None

    def compute_disparity(self):
        if self.left_image is None or self.right_image is None:
            return None

        # Ensure images are the same size
        if self.left_image.shape != self.right_image.shape:
            return None

        # Compute disparity map
        disparity = self.stereo.compute(self.left_image, self.right_image)

        # Convert to float and normalize
        disparity = disparity.astype(np.float32) / 16.0

        # Filter out invalid disparities
        disparity[disparity <= 0] = np.nan

        return disparity

    def disparity_to_3d(self, disparity):
        if disparity is None or self.focal_length is None or self.baseline is None:
            return None

        height, width = disparity.shape
        points_3d = []

        # Camera intrinsics
        cx = self.camera_matrix_left[0, 2]
        cy = self.camera_matrix_left[1, 2]
        fx = self.focal_length

        for v in range(height):
            for u in range(width):
                d = disparity[v, u]
                if not np.isnan(d) and d > 0:
                    # Convert pixel coordinates to 3D point
                    z = (fx * self.baseline) / d
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fx

                    # Filter points within range
                    if 0.1 < z < self.range_limit:
                        points_3d.append([x, y, z])

        return np.array(points_3d) if points_3d else None

    def analyze_terrain_traversability(self, points_3d):
        if points_3d is None or len(points_3d) == 0:
            return None

        # Initialize costmap
        costmap = np.full(
            (self.grid_height, self.grid_width), -1, dtype=np.int8
        )  # Unknown
        height_map = np.full((self.grid_height, self.grid_width), np.nan)

        # Convert 3D points to grid coordinates
        for point in points_3d:
            x, y, z = point

            # Transform to robot-centric coordinates (camera is assumed at robot center)
            # Adjust these transformations based on your camera mounting
            robot_x = z  # Forward direction
            robot_y = -x  # Left-right direction
            height = -y + self.robot_height  # Height relative to ground

            # Convert to grid coordinates
            grid_x = int((robot_x + self.grid_size / 2) / self.resolution)
            grid_y = int((robot_y + self.grid_size / 2) / self.resolution)

            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Update height map with maximum height in each cell
                if (
                    np.isnan(height_map[grid_y, grid_x])
                    or height > height_map[grid_y, grid_x]
                ):
                    height_map[grid_y, grid_x] = height

        # Analyze traversability based on height and slope
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if not np.isnan(height_map[y, x]):
                    cell_height = height_map[y, x]

                    # Check if too high (obstacle)
                    if cell_height > self.min_obstacle_height:
                        costmap[y, x] = 100  # Obstacle
                        continue

                    # Check slope with neighboring cells
                    max_slope = 0
                    neighbors_checked = 0

                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue

                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                if not np.isnan(height_map[ny, nx]):
                                    height_diff = abs(cell_height - height_map[ny, nx])
                                    distance = self.resolution * math.sqrt(
                                        dx * dx + dy * dy
                                    )
                                    slope = height_diff / distance
                                    max_slope = max(max_slope, slope)
                                    neighbors_checked += 1

                    # Determine traversability
                    if neighbors_checked > 0:
                        if max_slope > 0.5:  # Too steep (>26.6 degrees)
                            costmap[y, x] = 100  # Obstacle
                        elif max_slope > 0.3:  # Moderate slope
                            costmap[y, x] = 75  # High cost
                        elif max_slope > 0.1:  # Gentle slope
                            costmap[y, x] = 25  # Low cost
                        else:
                            costmap[y, x] = 0  # Free space
                    else:
                        costmap[y, x] = 0  # Free space if isolated point
                else:
                    # No height data - mark as unknown or free based on surrounding
                    surrounding_obstacles = 0
                    surrounding_cells = 0

                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                if costmap[ny, nx] >= 75:
                                    surrounding_obstacles += 1
                                surrounding_cells += 1

                    if (
                        surrounding_cells > 0
                        and surrounding_obstacles / surrounding_cells > 0.5
                    ):
                        costmap[y, x] = 50  # Likely obstacle area

        # Apply inflation to obstacles
        self.inflate_obstacles(costmap)

        return costmap

    def inflate_obstacles(self, costmap):
        inflation_radius = int(0.25 / self.resolution)  # 25cm inflation
        inflated = np.copy(costmap)

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if costmap[y, x] == 100:  # Obstacle
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                dist = math.sqrt(dx * dx + dy * dy)
                                if dist <= inflation_radius:
                                    inflation_cost = max(
                                        0, 90 - int(90 * dist / inflation_radius)
                                    )
                                    if (
                                        costmap[ny, nx] != 100
                                    ):  # Don't overwrite obstacles
                                        inflated[ny, nx] = max(
                                            inflated[ny, nx], inflation_cost
                                        )

        np.copyto(costmap, inflated)

    def create_pointcloud_msg(self, points_3d):
        if points_3d is None or len(points_3d) == 0:
            return None

        # Create PointCloud2 message
        header = self.get_clock().now().to_msg()

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for point in points_3d:
            x, y, z = point
            # Transform coordinates for ROS convention
            cloud_data.extend(struct.pack("fff", z, -x, -y + self.robot_height))

        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.stamp = header
        pointcloud_msg.header.frame_id = "odom"
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points_3d)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 12
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
        pointcloud_msg.data = bytes(cloud_data)
        pointcloud_msg.is_dense = True

        return pointcloud_msg

    def process_zed_data(self):
        # Try to get 3D points from pointcloud first, fallback to depth image
        points_3d = self.extract_points_from_pointcloud()
        if points_3d is None:
            points_3d = self.extract_points_from_depth()

        if points_3d is None:
            return

        # Publish point cloud for visualization (republish processed data)
        # pointcloud_msg = self.create_pointcloud_msg(points_3d)
        # if pointcloud_msg is not None:
        #     self.pointcloud_pub.publish(pointcloud_msg)

        # Create traversability costmap
        costmap = self.analyze_terrain_traversability(points_3d)
        if costmap is None:
            return

        # Publish costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "odom"

        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.grid_width
        costmap_msg.info.height = self.grid_height

        costmap_msg.info.origin = Pose()
        costmap_msg.info.origin.position.x = -self.grid_size / 2
        costmap_msg.info.origin.position.y = -self.grid_size / 2
        costmap_msg.info.origin.position.z = 0.0

        costmap_msg.data = costmap.flatten().tolist()

        self.costmap_pub.publish(costmap_msg)
        self.get_logger().info(f"Published ZED costmap with {len(points_3d)} 3D points")


def main(args=None):
    rclpy.init(args=args)
    node = StereoCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
