#!/usr/bin/env python3
"""
This script implements a LiDAR-based odometry node for ROS2.

It uses a vectorized implementation of the Generalized Iterative Closest Point (G-ICP)
algorithm to estimate the robot's motion by aligning consecutive point cloud scans.
The resulting pose is published as a standard odometry message.
"""
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation as R


class LidarOdometry(Node):
    """
    A ROS2 node for calculating odometry from LiDAR point clouds.

    This node subscribes to a PointCloud2 topic, performs G-ICP to find the
    relative transform between sequential scans, accumulates the transforms to
    maintain a global pose, and publishes the result as a nav_msgs/Odometry message.

    Attributes:
        pcd (o3d.geometry.PointCloud): An Open3D point cloud object, reused for processing.
        prev_points (numpy.ndarray): Stores the downsampled points from the previous scan, used as the target for ICP.
        max_iterations (int): The maximum number of iterations for the G-ICP algorithm.
        min_delta_err (float): The convergence threshold for G-ICP based on the norm of the twist.
        global_pose (numpy.ndarray): The 4x4 transformation matrix representing the estimated global pose of the robot.
        last_timestamp (builtin_interfaces.msg.Time): The timestamp of the previously processed message, used for calculating dt for velocity.
        estimates (list): A history of global pose matrices, used for calculating velocity.
        pcd_sub (rclpy.subscription.Subscription): The subscriber for the input point cloud topic.
        odom_pub (rclpy.publisher.Publisher): The publisher for the calculated odometry message.

    ROS Subscribers:
        /lidar/velodyne_points (sensor_msgs.msg.PointCloud2)

    ROS Publishers:
        /lidar/odom (nav_msgs.msg.Odometry)
    """

    def __init__(self):
        """
        Initializes the LidarOdometry node.

        Sets up initial state variables, parameters, and ROS publishers/subscribers.
        """
        super().__init__("lidar_odom")
        self.pcd = o3d.geometry.PointCloud()

        self.prev_points = []
        self.max_iterations = 30
        self.min_delta_err = 1e-6
        self.global_pose = np.eye(4)

        self.last_timestamp = None
        self.estimates = []

        self.pcd_sub = self.create_subscription(
            PointCloud2, "/lidar/velodyne_points", self.process_sequence, qos_profile=10
        )

        self.odom_pub = self.create_publisher(Odometry, "lidar/odom", 10)

        self.get_logger().info("Lidar Odometry Node Initialized.")

    def get_pc_from_ros2_pc2_msg(self, msg):
        """
        Converts a ROS2 PointCloud2 message to a structured NumPy array of XYZ points.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The input point cloud message.

        Returns:
            numpy.ndarray or None: A NumPy array of shape (N, 3) containing the points,
                                  or None if the conversion fails or results in an empty cloud.
        """
        self.xyz_array = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if self.xyz_array.shape[0] == 0:
            self.get_logger().error("Array is EMPTY after conversion.")
            return None

        is_finite = np.all(np.isfinite(self.xyz_array), axis=1)
        self.clean_xyz_array = self.xyz_array[is_finite]

        if self.clean_xyz_array.shape[0] == 0:
            self.get_logger().error("Array is EMPTY after cleaning NaN/Inf.")
            return None
        return self.clean_xyz_array

    def get_downsampled_pointcloud(self, points_np):
        """
        Downsamples a point cloud using a voxel grid filter to reduce computation.

        Args:
            points_np (numpy.ndarray): The input point cloud as an (N, 3) NumPy array.

        Returns:
            numpy.ndarray: The downsampled point cloud as a NumPy array.
        """
        if points_np.shape[0] == 0:
            self.get_logger().warn("Point cloud is empty, cannot downsample!")
            return np.array([])

        self.pcd.points = o3d.utility.Vector3dVector(points_np)
        downpcd = self.pcd.voxel_down_sample(voxel_size=0.2)
        return np.asarray(downpcd.points)

    def get_nearest_neighbours(self, source_points, target_points):
        """
        Finds the nearest neighbor in the target point cloud for each point in the source.

        Args:
            source_points (numpy.ndarray): The source points for which to find neighbors.
            target_points (numpy.ndarray): The target point cloud to search within.

        Returns:
            tuple[numpy.ndarray, numpy.ndarray]: A tuple containing arrays of distances
                                                 and indices of the nearest neighbors.
        """
        if target_points.shape[0] == 0:
            return np.array([]), np.array([])

        tree = KDTree(target_points)
        distances, indices = tree.query(source_points, k=1)
        return distances, indices

    def transformation_from_twist(self, twist):
        """
        Converts a 6-element twist vector [tx, ty, tz, rx, ry, rz] to a 4x4 transformation matrix.

        Args:
            twist (numpy.ndarray): A 6x1 or (6,) array representing the twist.

        Returns:
            numpy.ndarray: The corresponding 4x4 homogeneous transformation matrix.
        """
        twist = twist.flatten()
        rotation_vector = twist[3:6]
        translation_vector = twist[0:3]
        rotation = R.from_rotvec(rotation_vector)
        r_matrix = rotation.as_matrix()
        T_delta = np.eye(4)
        T_delta[0:3, 0:3] = r_matrix
        T_delta[0:3, 3] = translation_vector
        return T_delta

    def skew_symm_vectorized(self, m):
        """
        Efficiently creates a batch of skew-symmetric matrices from a batch of vectors.

        Args:
            m (numpy.ndarray): An (N, 3) array of vectors.

        Returns:
            numpy.ndarray: An (N, 3, 3) array of corresponding skew-symmetric matrices.
        """
        N = m.shape[0]
        ss_m = np.zeros((N, 3, 3))
        ss_m[:, 0, 1] = -m[:, 2]
        ss_m[:, 0, 2] = m[:, 1]
        ss_m[:, 1, 0] = m[:, 2]
        ss_m[:, 1, 2] = -m[:, 0]
        ss_m[:, 2, 0] = -m[:, 1]
        ss_m[:, 2, 1] = m[:, 0]
        return ss_m

    def genz_icp(self, source_points, target_points):
        """
        Performs Generalized-ICP to find the rigid transformation that aligns the source
        point cloud to the target point cloud.

        Args:
            source_points (numpy.ndarray): The source point cloud (the one that moves).
            target_points (numpy.ndarray): The target point cloud (the fixed reference).

        Returns:
            numpy.ndarray: The 4x4 transformation matrix that aligns source to target.
        """
        pcd_source = o3d.geometry.PointCloud()
        pcd_source.points = o3d.utility.Vector3dVector(source_points)
        pcd_source.estimate_covariances(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=20)
        )
        source_covs = np.asarray(pcd_source.covariances)

        pcd_target = o3d.geometry.PointCloud()
        pcd_target.points = o3d.utility.Vector3dVector(target_points)
        pcd_target.estimate_covariances(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=20)
        )
        target_covs = np.asarray(pcd_target.covariances)

        T = np.eye(4)
        transformed_source = source_points.copy()

        for iter_num in range(self.max_iterations):
            R_mat = T[:3, :3]
            distances, indices = self.get_nearest_neighbours(
                transformed_source, target_points
            )
            valid_mask = distances < 1.0
            if np.sum(valid_mask) < 20:
                self.get_logger().warn(
                    "G-ICP failed: Not enough valid correspondences."
                )
                return np.eye(4)

            s_corr, t_corr = (
                source_points[valid_mask],
                target_points[indices[valid_mask]],
            )
            s_cov_corr, t_cov_corr = (
                source_covs[valid_mask],
                target_covs[indices[valid_mask]],
            )

            num_corr = s_corr.shape[0]
            R_s_cov = np.einsum("ij,njk->nik", R_mat, s_cov_corr)
            M_corr = t_cov_corr + np.einsum("nij,kj->nik", R_s_cov, R_mat.T)
            M_inv_corr = np.linalg.inv(M_corr + np.eye(3) * 1e-8)

            J_corr = np.zeros((num_corr, 3, 6))
            J_corr[:, 0:3, 0:3] = np.eye(3)
            J_corr[:, :, 3:] = -self.skew_symm_vectorized(s_corr)

            d_corr = t_corr - transformed_source[valid_mask]

            temp_a = np.einsum("nji,njk->nik", J_corr, M_inv_corr)
            a_terms = np.einsum("nij,njk->nik", temp_a, J_corr)
            b_terms = np.einsum("nij,nj->ni", temp_a, d_corr)

            a_sum = np.sum(a_terms, axis=0)
            b_sum = np.sum(b_terms, axis=0).reshape(6, 1)

            try:
                delta_x, _, _, _ = np.linalg.lstsq(a_sum, b_sum, rcond=None)
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular matrix in G-ICP least squares solve.")
                break

            T_delta = self.transformation_from_twist(delta_x)
            T = T_delta @ T

            homog_source = np.hstack(
                (source_points, np.ones((source_points.shape[0], 1)))
            )
            transformed_source = (homog_source @ T.T)[:, :3]

            if np.linalg.norm(delta_x) < self.min_delta_err:
                self.get_logger().info(
                    f"G-ICP converged after {iter_num+1} iterations."
                )
                break
        return T

    def process_sequence(self, msg):
        """
        The main callback function that processes each incoming point cloud message.

        This method orchestrates the odometry pipeline: point cloud conversion,
        downsampling, ICP alignment, global pose update, and odometry message publishing.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The incoming point cloud message from the subscriber.
        """
        raw_points = self.get_pc_from_ros2_pc2_msg(msg)
        if raw_points is None:
            return

        down_points = self.get_downsampled_pointcloud(raw_points)
        if down_points.shape[0] < 50:
            self.get_logger().warn(
                "Not enough points after downsampling, skipping frame."
            )
            return

        if len(self.prev_points) == 0:
            self.get_logger().info("First frame received. Storing points as reference.")
            self.prev_points = down_points
            self.last_timestamp = msg.header.stamp
            return

        self.get_logger().info(
            f"Aligning current frame ({down_points.shape[0]} pts) to previous ({self.prev_points.shape[0]} pts)"
        )

        relative_transform = self.genz_icp(down_points, self.prev_points)

        self.global_pose = self.global_pose @ np.linalg.inv(relative_transform)

        self.estimates.append(self.global_pose.copy())

        current_timestamp = msg.header.stamp
        odom_msg = self.create_odom_message(self.global_pose, current_timestamp)
        self.odom_pub.publish(odom_msg)

        self.prev_points = down_points
        self.last_timestamp = current_timestamp

        current_position = self.global_pose[:3, 3]
        self.get_logger().info(f"Global Position (x,y,z): {current_position}")

    def create_odom_message(self, T, timestamp_msg):
        """
        Creates a nav_msgs/Odometry message from a global pose matrix and timestamp.

        Args:
            T (numpy.ndarray): The 4x4 global pose matrix.
            timestamp_msg (builtin_interfaces.msg.Time): The timestamp for the message header.

        Returns:
            nav_msgs.msg.Odometry: The populated odometry message.
        """
        odom = Odometry()
        odom.header.stamp = timestamp_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quat = R.from_matrix(rotation_matrix).as_quat()

        odom.pose.pose.position = Point(
            x=translation[0], y=translation[1], z=translation[2]
        )
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        if self.last_timestamp is not None and len(self.estimates) > 1:
            dt_duration = rclpy.time.Time.from_msg(
                timestamp_msg
            ) - rclpy.time.Time.from_msg(self.last_timestamp)
            dt = dt_duration.nanoseconds / 1e9

            if dt > 1e-6:
                prev_T = self.estimates[-2]
                prev_translation = prev_T[:3, 3]
                velocity = (translation - prev_translation) / dt
                odom.twist.twist.linear.x = float(velocity[0])
                odom.twist.twist.linear.y = float(velocity[1])
                odom.twist.twist.linear.z = float(velocity[2])

                prev_rotation = prev_T[:3, :3]
                rel_rotation = rotation_matrix @ prev_rotation.T
                rotvec = R.from_matrix(rel_rotation).as_rotvec()
                angular_velocity = rotvec / dt
                odom.twist.twist.angular.x = float(angular_velocity[0])
                odom.twist.twist.angular.y = float(angular_velocity[1])
                odom.twist.twist.angular.z = float(angular_velocity[2])

        return odom


def main(args=None):
    """
    Main entry point for the Lidar odometry node.
    """
    rclpy.init(args=args)
    lo = LidarOdometry()
    try:
        rclpy.spin(lo)
    except KeyboardInterrupt:
        pass
    finally:
        lo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
