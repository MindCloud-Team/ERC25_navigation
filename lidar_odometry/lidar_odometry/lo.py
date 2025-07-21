#!/home/ibrahim/venvs/rosenv/bin/python3    # Change To Your Default Interpreter
"""
Optimized LiDAR-based odometry node for ROS2.

This script combines your excellent custom G-ICP implementation with Open3D optimizations
for preprocessing and other operations. It keeps the vectorized G-ICP algorithm while
optimizing point cloud processing, downsampling, and correspondence finding.
"""
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
import time

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
        
        # State variables
        self.prev_cloud = None
        self.prev_points = None
        self.prev_normals = None
        self.prev_covariances = None
        self.global_pose = np.eye(4)
        self.pose_history = []
        self.timestamp_history = []
        
        # Performance monitoring
        self.processing_times = []
        self.frame_count = 0
        
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
    
    def ros_pointcloud_to_numpy(self, msg):
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
            
            # Vectorized filtering of finite points
            valid_mask = np.all(np.isfinite(xyz_array), axis=1)
            xyz_clean = xyz_array[valid_mask]
            
            if xyz_clean.shape[0] < 100:
                self.get_logger().warn(f"Too few valid points: {xyz_clean.shape[0]}")
                return None
            
            return xyz_clean
            
        except Exception as e:
            self.get_logger().error(f"Error converting point cloud: {e}")
            return None
    
    def preprocess_cloud_optimized(self, points_np):
        """
        Optimized preprocessing using Open3D for downsampling and outlier removal.
        
        Args:
            points_np (numpy.ndarray): Input points array
            
        Returns:
            tuple: (downsampled_points, normals, covariances) or None if failed
        """
        if points_np.shape[0] == 0:
            return None
        
        # Use pre-allocated cloud object for efficiency
        self.temp_cloud.points = o3d.utility.Vector3dVector(points_np)
        
        # Efficient voxel downsampling
        downsampled_cloud = self.temp_cloud.voxel_down_sample(self.voxel_size)
        
        if len(downsampled_cloud.points) < 50:
            self.get_logger().warn("Not enough points after downsampling")
            return None
        
        # Statistical outlier removal
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
        
        # Orient normals consistently
        clean_cloud.orient_normals_consistent_tangent_plane(k=10)
        
        # Estimate covariances for G-ICP
        clean_cloud.estimate_covariances(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=self.normal_max_nn
            )
        )
        
        # Convert back to numpy arrays
        points = np.asarray(clean_cloud.points)
        normals = np.asarray(clean_cloud.normals)
        covariances = np.asarray(clean_cloud.covariances)
        
        return points, normals, covariances
    
    def get_nearest_neighbours_optimized(self, source_points, target_points):
        """
        Optimized nearest neighbor search using KDTree.
        
        Args:
            source_points (numpy.ndarray): Source points
            target_points (numpy.ndarray): Target points
            
        Returns:
            tuple: (distances, indices) arrays
        """
        if target_points.shape[0] == 0:
            return np.array([]), np.array([])
        
        # Use scipy's optimized KDTree
        tree = KDTree(target_points)
        distances, indices = tree.query(source_points, k=1)
        return distances, indices
    
    def transformation_from_twist(self, twist):
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
        
        # Build transformation matrix
        T_delta = np.eye(4)
        T_delta[:3, :3] = rotation.as_matrix()
        T_delta[:3, 3] = twist[:3]
        
        return T_delta
    
    def skew_symmetric_batch(self, vectors):
        """
        Vectorized skew-symmetric matrix computation.
        
        Args:
            vectors (numpy.ndarray): (N, 3) array of vectors
            
        Returns:
            numpy.ndarray: (N, 3, 3) array of skew-symmetric matrices
        """
        N = vectors.shape[0]
        skew_matrices = np.zeros((N, 3, 3))
        
        # Vectorized assignment
        skew_matrices[:, 0, 1] = -vectors[:, 2]
        skew_matrices[:, 0, 2] = vectors[:, 1]
        skew_matrices[:, 1, 0] = vectors[:, 2]
        skew_matrices[:, 1, 2] = -vectors[:, 0]
        skew_matrices[:, 2, 0] = -vectors[:, 1]
        skew_matrices[:, 2, 1] = vectors[:, 0]
        
        return skew_matrices
    
    def generalized_icp_optimized(self, source_points, source_covs, target_points, target_covs):
        """
        Your optimized Generalized-ICP implementation.
        
        Args:
            source_points (numpy.ndarray): Source point cloud
            source_covs (numpy.ndarray): Source covariance matrices  
            target_points (numpy.ndarray): Target point cloud
            target_covs (numpy.ndarray): Target covariance matrices
            
        Returns:
            numpy.ndarray: 4x4 transformation matrix
        """
        T = np.eye(4)
        transformed_source = source_points.copy()
        
        for iteration in range(self.max_iterations):
            R_current = T[:3, :3]
            
            # Find correspondences
            distances, indices = self.get_nearest_neighbours_optimized(transformed_source, target_points)
            
            # Filter valid correspondences
            valid_mask = distances < self.max_correspondence_distance
            num_valid = np.sum(valid_mask)
            
            if num_valid < 20:
                self.get_logger().warn(f"G-ICP failed: Only {num_valid} valid correspondences")
                return np.eye(4)
            
            # Get corresponding points and covariances
            src_corr = source_points[valid_mask]
            tgt_corr = target_points[indices[valid_mask]]
            src_cov_corr = source_covs[valid_mask]
            tgt_cov_corr = target_covs[indices[valid_mask]]
            
            # Compute combined covariance matrices (G-ICP formulation)
            R_src_cov = np.einsum('ij,njk->nik', R_current, src_cov_corr)
            combined_cov = tgt_cov_corr + np.einsum('nij,kj->nik', R_src_cov, R_current.T)
            
            # Add regularization for numerical stability
            combined_cov += np.eye(3)[None, :, :] * 1e-8
            
            try:
                # Batch inverse of covariance matrices
                combined_cov_inv = np.linalg.inv(combined_cov)
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular covariance matrices in G-ICP")
                break
            
            # Build Jacobian matrices
            num_corr = src_corr.shape[0]
            J = np.zeros((num_corr, 3, 6))
            J[:, :, :3] = np.eye(3)  # Translation part
            J[:, :, 3:] = -self.skew_symmetric_batch(src_corr)  # Rotation part
            
            # Compute residuals
            residuals = tgt_corr - transformed_source[valid_mask]
            
            # Compute weighted least squares terms
            # A = sum(J^T * C^-1 * J), b = sum(J^T * C^-1 * r)
            JT_Cinv = np.einsum('nji,njk->nik', J, combined_cov_inv)  # J^T * C^-1
            A_terms = np.einsum('nij,njk->nik', JT_Cinv, J)  # (J^T * C^-1) * J
            b_terms = np.einsum('nij,nj->ni', JT_Cinv, residuals)  # (J^T * C^-1) * r
            
            A = np.sum(A_terms, axis=0)
            b = np.sum(b_terms, axis=0).reshape(-1, 1)
            
            # Solve for twist update
            try:
                delta_twist, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
                delta_twist = delta_twist.flatten()
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular matrix in G-ICP least squares")
                break
            
            # Convert twist to transformation and update
            T_delta = self.transformation_from_twist(delta_twist)
            T = T_delta @ T
            
            # Update transformed source points
            homog_source = np.hstack((source_points, np.ones((source_points.shape[0], 1))))
            transformed_source = (homog_source @ T.T)[:, :3]
            
            # Check convergence
            if np.linalg.norm(delta_twist) < self.min_delta_err:
                self.get_logger().info(f"G-ICP converged after {iteration+1} iterations")
                break
        
        return T
    
    def process_pointcloud(self, msg):
        """
        Main processing callback for incoming point clouds.
        
        Args:
            msg (sensor_msgs.msg.PointCloud2): Input point cloud message
        """
        start_time = time.time()
        
        # Convert ROS message to numpy array
        raw_points = self.ros_pointcloud_to_numpy(msg)
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
            self.timestamp_history.append(msg.header.stamp)
            self.pose_history.append(self.global_pose.copy())
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
        
        # Store history
        self.pose_history.append(self.global_pose.copy())
        self.timestamp_history.append(msg.header.stamp)
        
        # Limit history size
        max_history = 10
        if len(self.pose_history) > max_history:
            self.pose_history = self.pose_history[-max_history:]
            self.timestamp_history = self.timestamp_history[-max_history:]
        
        # Create and publish odometry message
        odom_msg = self.create_odometry_message(msg.header.stamp)
        self.odom_pub.publish(odom_msg)
        
        # Update previous data
        self.prev_points = current_points
        self.prev_normals = current_normals  
        self.prev_covariances = current_covariances
        
        # Performance monitoring
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.frame_count += 1
        
        # Log results
        position = self.global_pose[:3, 3]
        self.get_logger().info(
            f"Position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], "
            f"Points: {current_points.shape[0]}, Processing: {processing_time*1000:.1f}ms"
        )
    
    def create_odometry_message(self, timestamp):
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
        odom.pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Calculate velocities
        if len(self.pose_history) >= 2 and len(self.timestamp_history) >= 2:
            dt = (rclpy.time.Time.from_msg(self.timestamp_history[-1]) - 
                  rclpy.time.Time.from_msg(self.timestamp_history[-2])).nanoseconds / 1e9
            
            if dt > 1e-6:
                # Linear velocity
                prev_position = self.pose_history[-2][:3, 3]
                linear_vel = (position - prev_position) / dt
                
                odom.twist.twist.linear.x = float(linear_vel[0])
                odom.twist.twist.linear.y = float(linear_vel[1])
                odom.twist.twist.linear.z = float(linear_vel[2])
                
                # Angular velocity
                prev_rotation = self.pose_history[-2][:3, :3]
                relative_rotation = rotation_matrix @ prev_rotation.T
                angular_vel = R.from_matrix(relative_rotation).as_rotvec() / dt
                
                odom.twist.twist.angular.x = float(angular_vel[0])
                odom.twist.twist.angular.y = float(angular_vel[1])
                odom.twist.twist.angular.z = float(angular_vel[2])
        
        # Set covariance estimates
        pose_cov = np.eye(6) * 0.01  # Position and orientation uncertainty
        twist_cov = np.eye(6) * 0.1   # Velocity uncertainty
        
        odom.pose.covariance = pose_cov.flatten().tolist()
        odom.twist.covariance = twist_cov.flatten().tolist()
        
        return odom
    
    def log_performance_stats(self):
        """Log performance statistics periodically."""
        if len(self.processing_times) > 0:
            recent_times = self.processing_times[-50:]  # Last 50 frames
            avg_time = np.mean(recent_times)
            max_time = np.max(recent_times)
            min_time = np.min(recent_times)
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