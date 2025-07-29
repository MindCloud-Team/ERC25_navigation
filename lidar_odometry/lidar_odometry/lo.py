#!/usr/bin/env python3    # Change To Your Default Interpreter
"""
Optimized LiDAR-based odometry node for ROS2.

This script combines your excellent custom G-ICP implementation with Open3D optimizations
for preprocessing and other operations. It keeps the vectorized G-ICP algorithm while
optimizing point cloud processing, downsampling, and correspondence finding.

Additional optimizations added:
- Pre-allocated memory buffers to reduce garbage collection overhead
- Faster correspondence finding with cKDTree
- Better memory management and vectorized operations
- Fixed some bugs and improved numerical stability
"""
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree  # Faster C implementation instead of KDTree
import time
from sensor_msgs_py import point_cloud2
from math import sqrt
from typing import Optional, Tuple
import warnings

# Suppress annoying numerical warnings for cleaner output
warnings.filterwarnings('ignore', category=RuntimeWarning)

class OptimizedLidarOdometry(Node):
    """
    Optimized ROS2 node for calculating odometry from LiDAR point clouds.
    
    Uses your custom vectorized G-ICP implementation with Open3D optimizations
    for preprocessing operations like downsampling, normal estimation, and outlier removal.

    Attributes:
        prev_cloud (o3d.geometry.PointCloud): Previous preprocessed point cloud
        prev_points (numpy.ndarray): Previous point array for G-ICP
        prev_normals (numpy.ndarray): Previous normals for G-ICP
        prev_covariances (numpy.ndarray): Previous covariances for G-ICP
        global_pose (numpy.ndarray): 4x4 transformation matrix for global pose
        pose_history (list): History of poses for velocity calculation
        timestamp_history (list): History of timestamps
        
        New optimized stuff:
        - Pre-allocated memory buffers to avoid constant reallocation
        - Circular buffers for history instead of lists (way faster)
        - Better numerical stability checks
    """
    
    def __init__(self):
        """Initialize the optimized LiDAR odometry node."""
        super().__init__("optimized_lidar_odom")
        
        # Parameters - can be made configurable via ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('voxel_size', 0.2),
                ('normal_radius', 0.5),
                ('normal_max_nn', 20),
                ('max_iterations', 30),
                ('min_delta_err', 1e-6),
                ('max_correspondence_distance', 1.0),
                ('statistical_outlier_nb_neighbors', 20),
                ('statistical_outlier_std_ratio', 2.0),
                ('min_distance_filter_inches', 6.72),
                ('max_points_per_frame', 8000),  # Limit points for real-time performance
            ]
        )
        
        # Get parameters
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.normal_radius = self.get_parameter('normal_radius').get_parameter_value().double_value
        self.normal_max_nn = self.get_parameter('normal_max_nn').get_parameter_value().integer_value
        self.max_iterations = self.get_parameter('max_iterations').get_parameter_value().integer_value
        self.min_delta_err = self.get_parameter('min_delta_err').get_parameter_value().double_value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').get_parameter_value().double_value
        self.outlier_nb_neighbors = self.get_parameter('statistical_outlier_nb_neighbors').get_parameter_value().integer_value
        self.outlier_std_ratio = self.get_parameter('statistical_outlier_std_ratio').get_parameter_value().double_value
        self.min_distance_filter = self.get_parameter('min_distance_filter_inches').get_parameter_value().double_value
        self.max_points = self.get_parameter('max_points_per_frame').get_parameter_value().integer_value
        
        # State variables
        self.prev_cloud = None
        self.prev_points = None
        self.prev_normals = None
        self.prev_covariances = None
        self.global_pose = np.eye(4, dtype=np.float64)  # Use float64 for better precision
        
        # Optimized history storage using circular buffers instead of lists
        self.max_history = 10
        self.pose_history = np.zeros((self.max_history, 4, 4), dtype=np.float64)
        self.timestamp_history = [None] * self.max_history
        self.history_idx = 0
        self.history_count = 0
        
        # Performance monitoring with circular buffer
        self.processing_times = np.zeros(100)  # More efficient than list
        self.perf_idx = 0
        self.frame_count = 0
        
        # Pre-allocate commonly used matrices to avoid repeated allocation
        self.temp_transform = np.eye(4, dtype=np.float64)
        self.identity_3x3 = np.eye(3, dtype=np.float64)
        
        # Pre-allocate Open3D point cloud for efficiency
        self.temp_cloud = o3d.geometry.PointCloud()
        
        # ROS setup
        self.pcd_sub = self.create_subscription(
            PointCloud2,
            "/lidar/velodyne_points",
            self.process_pointcloud,
            qos_profile=10
        )
        
        self.odom_pub = self.create_publisher(Odometry, 'lidar/odom', 10)
        
        # Performance monitoring timer
        self.create_timer(5.0, self.log_performance_stats)
        
        self.get_logger().info("Optimized Lidar Odometry Node Initialized with Custom G-ICP")
    
    def ros_pointcloud_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Convert ROS2 PointCloud2 message to numpy array efficiently.
        
        Args:
            msg (sensor_msgs.msg.PointCloud2): Input point cloud message
            
        Returns:
            numpy.ndarray or None: Clean XYZ points array or None if conversion fails
        """
        try:
            # Extract XYZ points using sensor_msgs_py (optimized)
            xyz_array = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            if xyz_array.shape[0] == 0:
                self.get_logger().warn("Empty point cloud received")
                return None
            
            # Vectorized filtering of finite points (way faster than loops)
            valid_mask = np.all(np.isfinite(xyz_array), axis=1)
            xyz_clean = xyz_array[valid_mask]
            
            if xyz_clean.shape[0] < 100:
                self.get_logger().warn(f"Too few valid points: {xyz_clean.shape[0]}")
                return None
            
            # Limit points for real-time performance if needed
            if xyz_clean.shape[0] > self.max_points:
                # Random sampling to maintain point distribution
                indices = np.random.choice(xyz_clean.shape[0], self.max_points, replace=False)
                xyz_clean = xyz_clean[indices]
                self.get_logger().debug(f"Downsampled to {self.max_points} points for performance")
            
            return xyz_clean.astype(np.float64)  # Ensure consistent precision
            
        except Exception as e:
            self.get_logger().error(f"Error converting point cloud: {e}")
            return None
    
    def preprocess_cloud_optimized(self, points_np: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Optimized preprocessing using Open3D for downsampling and outlier removal.
        
        Args:
            points_np (numpy.ndarray): Input points array
            
        Returns:
            tuple: (downsampled_points, normals, covariances) or None if failed
        """
        if points_np.shape[0] == 0:
            return None
        
        # Use pre-allocated cloud object for efficiency (avoids repeated allocation)
        self.temp_cloud.points = o3d.utility.Vector3dVector(points_np)
        
        # Efficient voxel downsampling
        downsampled_cloud = self.temp_cloud.voxel_down_sample(self.voxel_size)
        
        if len(downsampled_cloud.points) < 50:
            self.get_logger().warn("Not enough points after downsampling")
            return None
        
        # Statistical outlier removal (only if we have enough points)
        if len(downsampled_cloud.points) > self.outlier_nb_neighbors:
            clean_cloud, _ = downsampled_cloud.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio
            )
        else:
            clean_cloud = downsampled_cloud
        
        if len(clean_cloud.points) < 20:
            self.get_logger().warn("Not enough points after outlier removal")
            return None
        
        # Estimate normals efficiently
        clean_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=self.normal_max_nn
            )
        )
        
        # Orient normals consistently (important for G-ICP)
        clean_cloud.orient_normals_consistent_tangent_plane(k=10)
        
        # Estimate covariances for G-ICP
        clean_cloud.estimate_covariances(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=self.normal_max_nn
            )
        )
        
        # Convert back to numpy arrays with consistent precision
        points = np.asarray(clean_cloud.points, dtype=np.float64)
        normals = np.asarray(clean_cloud.normals, dtype=np.float64)
        covariances = np.asarray(clean_cloud.covariances, dtype=np.float64)
        
        return points, normals, covariances
    
    def get_nearest_neighbours_optimized(self, source_points: np.ndarray, target_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Optimized nearest neighbor search using cKDTree (C implementation = faster).
        
        Args:
            source_points (numpy.ndarray): Source points
            target_points (numpy.ndarray): Target points
            
        Returns:
            tuple: (distances, indices) arrays
        """
        if target_points.shape[0] == 0:
            return np.array([]), np.array([])
        
        # Use scipy's optimized cKDTree (C implementation, much faster than KDTree)
        tree = cKDTree(target_points)
        distances, indices = tree.query(source_points, k=1, workers=-1)  # Use all CPU cores
        return distances, indices
    
    def transformation_from_twist(self, twist: np.ndarray) -> np.ndarray:
        """
        Efficient twist to transformation matrix conversion.
        
        Args:
            twist (numpy.ndarray): 6-element twist vector
            
        Returns:
            numpy.ndarray: 4x4 transformation matrix
        """
        twist = twist.flatten()
        
        # Use scipy's optimized rotation conversion
        rotation = R.from_rotvec(twist[3:6])
        
        # Build transformation matrix using pre-allocated matrix
        self.temp_transform.fill(0.0)
        self.temp_transform[:3, :3] = rotation.as_matrix()
        self.temp_transform[:3, 3] = twist[:3]
        self.temp_transform[3, 3] = 1.0
        
        return self.temp_transform.copy()
    
    def skew_symmetric_batch(self, vectors: np.ndarray) -> np.ndarray:
        """
        Vectorized skew-symmetric matrix computation (much faster than loops).
        
        Args:
            vectors (numpy.ndarray): (N, 3) array of vectors
            
        Returns:
            numpy.ndarray: (N, 3, 3) array of skew-symmetric matrices
        """
        N = vectors.shape[0]
        skew_matrices = np.zeros((N, 3, 3), dtype=np.float64)
        
        # Vectorized assignment (way faster than individual assignments)
        skew_matrices[:, 0, 1] = -vectors[:, 2]
        skew_matrices[:, 0, 2] = vectors[:, 1]
        skew_matrices[:, 1, 0] = vectors[:, 2]
        skew_matrices[:, 1, 2] = -vectors[:, 0]
        skew_matrices[:, 2, 0] = -vectors[:, 1]
        skew_matrices[:, 2, 1] = vectors[:, 0]
        
        return skew_matrices
    
    def generalized_icp_optimized(self, source_points: np.ndarray, source_covs: np.ndarray, 
                                 target_points: np.ndarray, target_covs: np.ndarray) -> np.ndarray:
        """
        Your optimized Generalized-ICP implementation with additional stability improvements.
        
        Args:
            source_points (numpy.ndarray): Source point cloud
            source_covs (numpy.ndarray): Source covariance matrices  
            target_points (numpy.ndarray): Target point cloud
            target_covs (numpy.ndarray): Target covariance matrices
            
        Returns:
            numpy.ndarray: 4x4 transformation matrix
        """
        T = np.eye(4, dtype=np.float64)
        transformed_source = source_points.copy()
        
        # Pre-compute homogeneous coordinate matrix for efficiency
        homog_ones = np.ones((source_points.shape[0], 1), dtype=np.float64)
        
        for iteration in range(self.max_iterations):
            R_current = T[:3, :3]
            
            # Find correspondences using optimized cKDTree
            distances, indices = self.get_nearest_neighbours_optimized(transformed_source, target_points)
            
            # Filter valid correspondences
            valid_mask = distances < self.max_correspondence_distance
            num_valid = np.sum(valid_mask)
            
            if num_valid < 20:
                self.get_logger().warn(f"G-ICP failed: Only {num_valid} valid correspondences")
                return np.eye(4, dtype=np.float64)
            
            # Get corresponding points and covariances
            src_corr = source_points[valid_mask]
            tgt_corr = target_points[indices[valid_mask]]
            src_cov_corr = source_covs[valid_mask]
            tgt_cov_corr = target_covs[indices[valid_mask]]
            
            # Compute combined covariance matrices (G-ICP formulation)
            R_src_cov = np.einsum('ij,njk->nik', R_current, src_cov_corr)
            combined_cov = tgt_cov_corr + np.einsum('nij,kj->nik', R_src_cov, R_current.T)
            
            # Add regularization for numerical stability (important!)
            regularization = np.eye(3, dtype=np.float64) * 1e-6  # Increased for stability
            combined_cov += regularization[None, :, :]
            
            try:
                # Batch inverse of covariance matrices with better error handling
                combined_cov_inv = np.linalg.inv(combined_cov)
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular covariance matrices in G-ICP")
                break
            
            # Build Jacobian matrices
            num_corr = src_corr.shape[0]
            J = np.zeros((num_corr, 3, 6), dtype=np.float64)
            J[:, :, :3] = self.identity_3x3  # Translation part (reuse identity matrix)
            J[:, :, 3:] = -self.skew_symmetric_batch(src_corr)  # Rotation part
            
            # Compute residuals
            residuals = tgt_corr - transformed_source[valid_mask]
            
            # Compute weighted least squares terms (vectorized for speed)
            # A = sum(J^T * C^-1 * J), b = sum(J^T * C^-1 * r)
            JT_Cinv = np.einsum('nji,njk->nik', J, combined_cov_inv)  # J^T * C^-1
            A_terms = np.einsum('nij,njk->nik', JT_Cinv, J)  # (J^T * C^-1) * J
            b_terms = np.einsum('nij,nj->ni', JT_Cinv, residuals)  # (J^T * C^-1) * r
            
            A = np.sum(A_terms, axis=0)
            b = np.sum(b_terms, axis=0).reshape(-1, 1)
            
            # Add regularization to A matrix for numerical stability
            A += np.eye(6, dtype=np.float64) * 1e-8
            
            # Solve for twist update with better numerical handling
            try:
                delta_twist, residuals_norm, rank, s = np.linalg.lstsq(A, b, rcond=1e-6)
                delta_twist = delta_twist.flatten()
                
                # Check for numerical issues
                if not np.all(np.isfinite(delta_twist)):
                    self.get_logger().warn("Non-finite twist update in G-ICP")
                    break
                    
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular matrix in G-ICP least squares")
                break
            
            # Convert twist to transformation and update
            T_delta = self.transformation_from_twist(delta_twist)
            T = T_delta @ T
            
            # Update transformed source points efficiently
            homog_source = np.hstack((source_points, homog_ones))
            transformed_source = (homog_source @ T.T)[:, :3]
            
            # Check convergence
            twist_norm = np.linalg.norm(delta_twist)
            if twist_norm < self.min_delta_err:
                self.get_logger().debug(f"G-ICP converged after {iteration+1} iterations")
                break
        
        return T
    
    def filter_by_distance(self, points_msg: PointCloud2) -> Optional[PointCloud2]:
        """
        Filter points by distance to remove too-close points.
        Optimized with vectorized operations instead of loops.
        
        Args:
            points_msg: Input PointCloud2 message
            
        Returns:
            Filtered PointCloud2 message or None if all points filtered
        """
        try:
            # Use the optimized read_points_numpy for direct structured array access
            points_structured = pc2.read_points_numpy(points_msg, field_names=("x", "y", "z"), skip_nans=True)
            
            if points_structured.shape[0] == 0:
                self.get_logger().warn("No valid points in input cloud")
                return None
            
            # Extract x, y, z coordinates from structured array (correct way)
            if points_structured.dtype.names:
                # It's a structured array with named fields
                x_coords = points_structured['x'].astype(np.float64)
                y_coords = points_structured['y'].astype(np.float64) 
                z_coords = points_structured['z'].astype(np.float64)
                # Stack into regular numpy array for vectorized operations
                points_np = np.column_stack((x_coords, y_coords, z_coords))
            else:
                # It's already a regular array
                points_np = points_structured.astype(np.float64)
            
            # Vectorized distance calculation (much faster than individual sqrt calls)
            distances = np.linalg.norm(points_np, axis=1)
            distances_inches = distances * 100 / 2.54  # Convert m -> cm -> inches
            
            # Filter points efficiently
            valid_mask = distances_inches > self.min_distance_filter
            far_points = points_np[valid_mask]
            
            if len(far_points) == 0:
                self.get_logger().warn("All points filtered out by distance!")
                return None
            
            # Debug info
            filtered_count = len(far_points)
            total_count = len(points_np)
            self.get_logger().debug(f"Distance filter: {filtered_count}/{total_count} points kept")
            
            # Create new PointCloud2 message from filtered points
            header = Header()
            header.stamp = points_msg.header.stamp
            header.frame_id = points_msg.header.frame_id
            
            filtered_msg = point_cloud2.create_cloud_xyz32(header, far_points.tolist())
            return filtered_msg
            
        except Exception as e:
            self.get_logger().error(f"Error in distance filtering: {e}")
            # Fallback to your original method if structured array approach fails
            try:
                far_points = []
                for point in point_cloud2.read_points(points_msg, field_names=("x", "y", "z"), skip_nans=True):
                    x, y, z = point
                    distance = sqrt(x**2 + y**2 + z**2)
                    distance_cm = distance * 100
                    distance_inch = distance_cm / 2.54

                    if distance_inch > self.min_distance_filter:
                        far_points.append((x, y, z))

                if len(far_points) == 0:
                    self.get_logger().warn("All points filtered out!")
                    return None

                # Create new PointCloud2 message from far points
                header = Header()
                header.stamp = points_msg.header.stamp
                header.frame_id = points_msg.header.frame_id

                filtered_msg = point_cloud2.create_cloud_xyz32(header, far_points)
                return filtered_msg
            except Exception as fallback_e:
                self.get_logger().error(f"Fallback filtering also failed: {fallback_e}")
                return None
    
    def add_to_history(self, pose: np.ndarray, timestamp) -> None:
        """
        Add pose and timestamp to circular buffer history.
        More efficient than list operations.
        """
        self.pose_history[self.history_idx] = pose.copy()
        self.timestamp_history[self.history_idx] = timestamp
        self.history_idx = (self.history_idx + 1) % self.max_history
        self.history_count = min(self.history_count + 1, self.max_history)
    
    def get_previous_pose(self) -> Optional[np.ndarray]:
        """Get the previous pose from circular buffer."""
        if self.history_count < 2:
            return None
        prev_idx = (self.history_idx - 2) % self.max_history
        return self.pose_history[prev_idx]
    
    def get_previous_timestamp(self):
        """Get the previous timestamp from circular buffer."""
        if self.history_count < 2:
            return None
        prev_idx = (self.history_idx - 2) % self.max_history
        return self.timestamp_history[prev_idx]
    
    def process_pointcloud(self, msg: PointCloud2) -> None:
        """
        Main processing callback for incoming point clouds.
        
        Args:
            msg (sensor_msgs.msg.PointCloud2): Input point cloud message
        """
        start_time = time.time()
        
        # Filter by distance first to reduce processing load
        filtered_pointcloud = self.filter_by_distance(msg)
        if filtered_pointcloud is None:
            return
        
        # Convert ROS message to numpy array
        raw_points = self.ros_pointcloud_to_numpy(filtered_pointcloud)
        if raw_points is None:
            return
        
        # Preprocess the point cloud
        preprocess_result = self.preprocess_cloud_optimized(raw_points)
        if preprocess_result is None:
            return
        
        current_points, current_normals, current_covariances = preprocess_result
        
        # First frame initialization
        if self.prev_points is None:
            self.get_logger().info(f"Initializing with first frame: {current_points.shape[0]} points")
            self.prev_points = current_points
            self.prev_normals = current_normals
            self.prev_covariances = current_covariances
            self.add_to_history(self.global_pose, msg.header.stamp)
            return
        
        # Perform G-ICP alignment
        self.get_logger().debug(
            f"G-ICP: {current_points.shape[0]} -> {self.prev_points.shape[0]} points"
        )
        
        relative_transform = self.generalized_icp_optimized(
            current_points, current_covariances,
            self.prev_points, self.prev_covariances
        )
        
        # Update global pose (inverse transform for proper odometry)
        self.global_pose = self.global_pose @ np.linalg.inv(relative_transform)
        
        # Store history using circular buffer
        self.add_to_history(self.global_pose, msg.header.stamp)
        
        # Create and publish odometry message
        odom_msg = self.create_odometry_message(msg.header.stamp)
        self.odom_pub.publish(odom_msg)
        
        # Update previous data
        self.prev_points = current_points
        self.prev_normals = current_normals  
        self.prev_covariances = current_covariances
        
        # Performance monitoring with circular buffer
        processing_time = time.time() - start_time
        self.processing_times[self.perf_idx] = processing_time
        self.perf_idx = (self.perf_idx + 1) % len(self.processing_times)
        self.frame_count += 1
        
        # Log results
        position = self.global_pose[:3, 3]
        self.get_logger().info(
            f"Position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], "
            f"Points: {current_points.shape[0]}, Processing: {processing_time*1000:.1f}ms"
        )
    
    def create_odometry_message(self, timestamp) -> Odometry:
        """
        Create nav_msgs/Odometry message from current pose and velocity.
        
        Args:
            timestamp (builtin_interfaces.msg.Time): Message timestamp
            
        Returns:
            nav_msgs.msg.Odometry: Odometry message
        """
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Extract position and orientation
        position = self.global_pose[:3, 3]
        rotation_matrix = self.global_pose[:3, :3]
        
        # Convert to quaternion
        quat = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
        
        # Set pose
        odom.pose.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        odom.pose.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        
        # Calculate velocities using circular buffer
        prev_pose = self.get_previous_pose()
        prev_timestamp = self.get_previous_timestamp()
        
        if prev_pose is not None and prev_timestamp is not None:
            dt = (rclpy.time.Time.from_msg(timestamp) - 
                  rclpy.time.Time.from_msg(prev_timestamp)).nanoseconds / 1e9
            
            if dt > 1e-6:
                # Linear velocity
                prev_position = prev_pose[:3, 3]
                linear_vel = (position - prev_position) / dt
                
                odom.twist.twist.linear.x = float(linear_vel[0])
                odom.twist.twist.linear.y = float(linear_vel[1])
                odom.twist.twist.linear.z = float(linear_vel[2])
                
                # Angular velocity
                prev_rotation = prev_pose[:3, :3]
                relative_rotation = rotation_matrix @ prev_rotation.T
                angular_vel = R.from_matrix(relative_rotation).as_rotvec() / dt
                
                odom.twist.twist.angular.x = float(angular_vel[0])
                odom.twist.twist.angular.y = float(angular_vel[1])
                odom.twist.twist.angular.z = float(angular_vel[2])
        
        # Set covariance estimates (you might want to tune these)
        pose_cov = np.eye(6) * 0.01  # Position and orientation uncertainty
        twist_cov = np.eye(6) * 0.1   # Velocity uncertainty
        
        odom.pose.covariance = pose_cov.flatten().tolist()
        odom.twist.covariance = twist_cov.flatten().tolist()
        
        return odom
    
    def log_performance_stats(self) -> None:
        """Log performance statistics periodically."""
        if self.frame_count > 0:
            # Use only valid entries from circular buffer
            valid_times = self.processing_times[:min(self.frame_count, len(self.processing_times))]
            
            if len(valid_times) > 0:
                avg_time = np.mean(valid_times)
                max_time = np.max(valid_times)
                min_time = np.min(valid_times)
                hz = 1.0 / avg_time if avg_time > 0 else 0
                
                self.get_logger().info(
                    f"Performance Stats - Frames: {self.frame_count}, "
                    f"Avg: {avg_time*1000:.1f}ms, Min: {min_time*1000:.1f}ms, "
                    f"Max: {max_time*1000:.1f}ms, Rate: {hz:.1f}Hz"
                )


def main(args=None):
    """Main entry point for the optimized LiDAR odometry node."""
    rclpy.init(args=args)
    
    node = OptimizedLidarOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
