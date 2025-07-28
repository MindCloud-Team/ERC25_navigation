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
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

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

def pnt2pnt_icp(self, points):
        mse = 1.0e6
        delta_mse = 1.0e6
        T = self.initial_T
        num_iterations = 0
        T_step = np.eye(4)
        transformed_source = np.copy(points)
        while delta_mse > self.min_delta_err and num_iterations < self.max_iterations:
                distances, neighbours, matched_target_points = self.get_nearest_neighbours(transformed_source)
                new_T = self.get_transformation(points, matched_target_points)
                if new_T is None:
                    self.get_logger().warn("Skipping ICP iteration: transformation is None.")
                    return  # or continue, or break
                
                source_h = np.hstack((points, np.ones((points.shape[0], 1))))
                transformed_source = (source_h @ new_T.T)[:, :3]

                T_step = new_T # The final transform is the one from the last successful iteration

                new_error = 0
                for i in  range(len(neighbours)):
                    diff = transformed_source[i,:self.m] - self.prev_points[neighbours[i][0],:3]
                    new_error += np.linalg.norm(diff) ** 2
                new_error /= float(len(matched_target_points))
                delta_mse = abs(mse - new_error)
                mse = new_error 
                num_iterations += 1
        #transformed_points = (points_h @ self.global_pose.T)[:, :3]
        
        pcd_current = o3d.geometry.PointCloud()
        pcd_current.points = o3d.utility.Vector3dVector(points)
        pcd_prev = o3d.geometry.PointCloud()
        pcd_prev.points = o3d.utility.Vector3dVector(self.prev_points)
        pcd_prev.paint_uniform_color([1, 0, 0])  
        pcd_current.paint_uniform_color([0, 1, 0])
        o3d.visualization.draw_geometries([pcd_prev,pcd_current])  
        self.prev_points = points  # update for next frame   
        self.assign_normals()   
        return T_step