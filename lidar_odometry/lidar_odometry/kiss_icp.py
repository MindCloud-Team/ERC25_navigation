#!/home/ibrahim/venvs/rosenv/bin/python3

from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import ctypes
import sensor_msgs
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
import sys



class LidarOdometry(Node):
    def __init__(self):
        super().__init__("lidar_odom")
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_sub = self.create_subscription(PointCloud2,
                                                 "/lidar/velodyne_points",
                                                  callback=self.get_pc_from_ros2_pc2_msg,
                                                  qos_profile=10)

        self.prev_points = []
        self.initial_T = np.eye(4)
        self.max_iterations = 100
        self.min_delta_err = 1e-6
    def get_pc_from_ros2_pc2_msg(self,msg):
        """ Returns point-cloud as a structured numpy array. 
        Note: can be used with any topic of message type 'sensor_msgs/PointCloud2'
        """
        self.xyz_array = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        # self.get_logger().info(f"Got point cloud with shape {self.xyz_array.shape}")
        # self.get_logger().info(f"x: {self.xyz_array[0]}")
        # self.get_logger().info(f"y: {self.xyz_array[1]}")
        # self.get_logger().info(f"z: {self.xyz_array[2]}")
        if self.xyz_array.shape[0] == 0:
                    self.get_logger().error("Array is EMPTY after conversion. No points to process.")
                    self.has_printed = True
                    return
     # Check for NaN and Infinity values - THIS IS THE MOST LIKELY PROBLEM
        has_nan = np.isnan(self.xyz_array).any()
        has_inf = np.isinf(self.xyz_array).any()
        self.get_logger().info(f"Does array contain NaN values? {has_nan}")
        self.get_logger().info(f"Does array contain Infinity values? {has_inf}")

            # If we have bad values, let's see how many
        if has_nan:
                nan_count = np.isnan(self.xyz_array).sum()
                self.get_logger().warn(f"Found {nan_count} NaN values.")
        if has_inf:
                inf_count = np.isinf(self.xyz_array).sum()
                self.get_logger().warn(f"Found {inf_count} Infinity values.")

            # Let's clean the data by removing rows with NaN or Inf
        self.clean_xyz_array = self.xyz_array[~np.isnan(self.xyz_array).any(axis=1)]
        self.clean_xyz_array = self.clean_xyz_array[~np.isinf(self.clean_xyz_array).any(axis=1)]
            
        self.get_logger().info(f"Shape after cleaning NaN/Inf: {self.clean_xyz_array.shape}")
        if self.clean_xyz_array.shape[0] == 0:
                self.get_logger().error("Array is EMPTY after cleaning. All points were invalid.")
                self.has_printed = True
                return
                
            # Check the bounds of the CLEAN data
        min_bounds = np.min(self.clean_xyz_array, axis=0)
        max_bounds = np.max(self.clean_xyz_array, axis=0)
        self.get_logger().info(f"Bounds of CLEAN data (min): {min_bounds}")
        self.get_logger().info(f"Bounds of CLEAN data (max): {max_bounds}")

            # Check if all points are identical (no volume)
        if np.all(min_bounds == max_bounds):
                self.get_logger().error("All points in the cloud are at the exact same location!")
                self.has_printed = True
                return   
           
        self.get_downsampled_pointcloud()
    
    def get_downsampled_pointcloud(self):

        if len(self.clean_xyz_array) == 0:
            self.get_logger().warn("Point cloud is empty!")
            return

        self.pcd.points = o3d.utility.Vector3dVector(self.clean_xyz_array)
        downpcd = self.pcd.voxel_down_sample(voxel_size=0.2)
        down_points = np.asarray(downpcd.points)
        #o3d.visualization.draw_geometries([downpcd])
        if(len(self.prev_points) == 0):
            self.prev_points = down_points
            return
        else:
            self.icp(down_points)

    
    def get_nearest_neighbours(self, points):
           tree = KDTree(self.prev_points)
           distances, neighbours = tree.query(points)
        #    self.get_logger().info(f"Sample distances: {distances[:5]}")
        #    self.get_logger().info(f"Indices of nearest neighbours: {neighbours[:5]}")
           matched_target_pts = []
           for i in range(len(neighbours)):
                matched_target_pts.append(self.prev_points[neighbours[i]])
           
           matched_target_pts = np.array(matched_target_pts)
           return distances, neighbours, matched_target_pts

    def get_transformation(self, points, matched_target_points):
        self.prev_points_arr = np.asarray(self.prev_points)
        if points.shape[1] != matched_target_points.shape[1]:
            self.get_logger().warn("Dimension mismatch between current and matched target points.")
            return None
        self.m = matched_target_points.shape[1]
        prev_centroid = np.mean(matched_target_points, axis=0)
        current_centroid = np.mean(points,axis=0)
        current_translated_points = points - current_centroid
        prev_translated_points = matched_target_points - prev_centroid
        covariance_matrix = np.dot(current_translated_points.T, prev_translated_points)
        U, S, VT = np.linalg.svd(covariance_matrix)
        R = np.dot(VT.T, U.T)
        if(np.linalg.det(R) < 0):
            VT[self.m-1,:] *= -1
            R = np.dot(VT.T, U.T)
        t = prev_centroid.reshape(-1,1) - np.dot(R, current_centroid.reshape(-1,1))
        T = np.eye(self.m+1)
        T[:self.m, :self.m] = R
        T[:self.m, -1] = t.ravel()
        return T

    def icp(self, points):
        mse = 1.0e6
        delta_mse = 1.0e6
        T = self.initial_T
        num_iterations = 0
        while delta_mse > self.min_delta_err and num_iterations < self.max_iterations:
                distances, neighbours, matched_target_points = self.get_nearest_neighbours(points)
                new_T = self.get_transformation(points, matched_target_points)
                if new_T is None:
                    self.get_logger().warn("Skipping ICP iteration: transformation is None.")
                    return  # or continue, or break
                T = np.dot(T,new_T)
                
                points_h = np.hstack((points, np.ones((points.shape[0], 1))))
                points = (points_h @ T.T)[:, :3]

                new_error = 0
                for i in  range(len(neighbours)):
                    diff = points[i,:self.m] - self.prev_points_arr[neighbours[i],:3]
                    new_error += np.dot(diff,diff.T)
                new_error /= float(len(matched_target_points))
                delta_mse = abs(mse - new_error)
                mse = new_error 
                num_iterations += 1
        points_h = np.hstack((points, np.ones((points.shape[0], 1))))  
        transformed_points = (points_h @ T.T)[:, :3]          
        pcd_current = o3d.geometry.PointCloud()
        pcd_current.points = o3d.utility.Vector3dVector(transformed_points)
        pcd_prev = o3d.geometry.PointCloud()
        pcd_prev.points = o3d.utility.Vector3dVector(self.prev_points)
        pcd_prev.paint_uniform_color([1, 0, 0])  
        pcd_current.paint_uniform_color([0, 1, 0])
        o3d.visualization.draw_geometries([pcd_prev,pcd_current])   
        return T
def main(args = None):

    """
    Main Entry point of the Lidar Odometry node
    """

    print("Python path:", sys.executable)
    print("Python version:", sys.version)

    rclpy.init(args=args) # Initializing ROS connection
    
    lo = LidarOdometry() # Initializing ROS node
    
    rclpy.spin(lo) # Spinning the node
    
    rclpy.shutdown() # Shutting down ROS connection

if __name__ == "__main__":
    main()