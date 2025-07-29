#! /usr/bin/env python3
"""
A Sample for Localization of the rover using the Zed camera

This sample predicts the position and orientation of the robot
using the Zed-D camera with IMU fusion
"""

import os
import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Imu

class ZedVisualOdometry(Node):
    def __init__(self):
        super().__init__("oak_visual_odom")

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.bridge = CvBridge()
        self.orb = cv2.ORB.create(3000)
        self.index_params = dict(algorithm=6, trees=5)
        self.search_params = dict(checks=50)

        self.K = None
        self.C_k = np.eye(4)
        self.last_timestamp = None
        self.prev_rgb_img = None
        self.prev_depth_img = None
        self.estimates = []
        self.frame_index = 0

        self.calib_sub = self.create_subscription(
            CameraInfo,
            "/left_cam/zed_node/rgb/camera_info",
            self.camera_info_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/left_cam/zed_node/imu/data',
            self.imu_callback,
            200
        )

        self.imu_queue = []

        self.get_logger().info("OAK Visual Odometry node initialized")

    def calc_essential_matrix(self, pts1, pts2):
        E, mask = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=0.5)
        _, R_mat, T, mask = cv2.recoverPose(E, pts1, pts2, self.K)
        T_mat = np.eye(4)
        T_mat[:3, :3] = R_mat
        T_mat[:3, 3] = T.flatten()
        return T_mat

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera calibration received")
            self.setup_synchronized_subscribers()
            self.destroy_subscription(self.calib_sub)

    def setup_synchronized_subscribers(self):
        self.rgb_sub = message_filters.Subscriber(self, Image, '/left_cam/zed_node/rgb/image_rect_color')
        self.ts = message_filters.ApproximateTimeSynchronizer([
            self.rgb_sub
        ], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)
        self.get_logger().info("Synchronized subscribers set up")

    def synchronized_callback(self, rgb_msg):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "mono8")
            if self.prev_rgb_img is not None:
                self.process_frames(self.prev_rgb_img, rgb_img, rgb_msg.header.stamp)
            self.prev_rgb_img = rgb_img
            self.last_timestamp = rgb_msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error in synchronized callback: {str(e)}")

    def process_frames(self, prev_rgb, curr_rgb, timestamp):
        i = self.frame_index
        left = curr_rgb
        if self.prev_rgb_img is None:
            self.prev_rgb_img = left
            return

        pts1, pts2, good_matches = self.match_features(self.prev_rgb_img, left)

        if pts1 is not None and len(pts1) > 8:
            T_k = self.calc_essential_matrix(pts1, pts2)

            # Initialize IMU data variables
            imu_times = np.array([])
            accs = np.array([])
            gyros = np.array([])
            
            # Get IMU data for the current time interval
            if self.last_timestamp is not None:
                imu_times, accs, gyros = self.integrate_imu(self.last_timestamp, timestamp)

            # Scale estimation using IMU or visual information
            if self.frame_index > 1:
                prev_pose = self.estimates[-1]
                prev_prev_pose = self.estimates[-2]

                # Try to use IMU for scale estimation
                if imu_times.shape[0] > 2:
                    dt = imu_times[-1] - imu_times[0]
                    # Remove gravity (assuming robot moves on roughly horizontal plane)
                    gravity_compensated_acc = accs.copy()
                    gravity_compensated_acc[:, 2] -= 9.81  # Remove gravity in Z direction
                    
                    velocity = np.mean(gravity_compensated_acc, axis=0) * dt  # crude integration
                    scale = np.linalg.norm(velocity) * dt
                    
                    # Clamp scale to reasonable values to avoid outliers
                    scale = np.clip(scale, 0.001, 2.0)
                    T_k[:3, 3] *= scale
                    print(f"IMU-derived scale at frame {i}: {scale:.4f}")
                else:
                    print("Insufficient IMU samples — falling back to visual scale")
                    prev_delta = prev_pose[:3, 3] - prev_prev_pose[:3, 3]
                    denom = np.linalg.norm(T_k[:3, 3])
                    scale = np.linalg.norm(prev_delta) / (denom + 1e-8)
                    # Clamp scale to reasonable values
                    scale = np.clip(scale, 0.001, 2.0)
                    T_k[:3, 3] *= scale
                    print(f"Visual scale at frame {i}: {scale:.4f}")
            else:
                # For first few frames, use a default small scale
                default_scale = 0.1
                T_k[:3, 3] *= default_scale
                print(f"Default scale at frame {i}: {default_scale:.4f}")

            # Validate motion estimates
            translation_norm = np.linalg.norm(T_k[:3, 3])
            rotation_angle = np.linalg.norm(R.from_matrix(T_k[:3, :3]).as_rotvec())

            if translation_norm > 1.0 or rotation_angle > np.pi / 2:
                self.get_logger().warn(f"Unrealistic motion at frame {i}: ∆={translation_norm:.3f}, θ={rotation_angle:.2f} rad — skipping")
                return

            # Update pose
            self.C_k = self.C_k @ T_k
            msg = self.create_odom_message(self.C_k, self.get_clock().now().to_msg())
            self.odom_pub.publish(msg)

            print(f"Pose at frame {i+1}:\n{self.C_k}\n")
            self.estimates.append(self.C_k.copy())
            self.frame_index += 1

    def match_features(self, img1, img2):
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)

        if des1 is None or des2 is None or len(des1) < 2 or len(des2) < 2:
            self.get_logger().warn("Not enough features detected")
            return None, None, None

        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        good_matches = []
        for m_n in matches:
            if len(m_n) != 2:
                continue
            m, n = m_n
            if m.distance < 0.8 * n.distance:
                good_matches.append(m)

        self.get_logger().info(f"Number of good matches: {len(good_matches)}")

        if len(good_matches) < 8:
            return None, None, None

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])

        return pts1, pts2, good_matches

    def create_odom_message(self, T, timestamp):
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position = Point(
            x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))
        odom.pose.pose.orientation = Quaternion(
            x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]), w=float(quaternion[3]))

        # Calculate velocity if we have previous estimates
        if self.last_timestamp is not None and len(self.estimates) > 1:
            dt = (timestamp.sec - self.last_timestamp.sec) + \
                 (timestamp.nanosec - self.last_timestamp.nanosec) * 1e-9

            if dt > 0.001:
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

    def visualize_trajectory(self):
        if len(self.estimates) < 2:
            self.get_logger().warn("Not enough poses to visualize trajectory")
            return

        x = [pose[0, 3] for pose in self.estimates]
        z = [pose[2, 3] for pose in self.estimates]

        plt.figure(figsize=(10, 5))
        plt.plot(x, z, label="Estimated", color='r')
        plt.legend()
        plt.xlabel("X Position")
        plt.ylabel("Z Position")
        plt.title("Visual Odometry Trajectory")
        plt.grid(True)
        plt.savefig(os.path.expanduser('~/ros_ws/trajectory.png'))
        self.get_logger().info("Trajectory visualization saved to trajectory.png")

    def imu_callback(self, msg):
        self.imu_queue.append(msg)

        # Keep only recent IMU data to limit memory usage
        if len(self.imu_queue) > 1000:
            self.imu_queue.pop(0)

    def integrate_imu(self, start_stamp, end_stamp):
        imu_times = []
        accs = []
        gyros = []

        start_time = start_stamp.sec + start_stamp.nanosec * 1e-9
        end_time = end_stamp.sec + end_stamp.nanosec * 1e-9

        for imu in self.imu_queue:
            t = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
            if start_time <= t <= end_time:
                imu_times.append(t)
                accs.append([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
                gyros.append([imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z])

        return np.array(imu_times), np.array(accs), np.array(gyros)


def main(args=None):
    rclpy.init(args=args)
    node = ZedVisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.visualize_trajectory()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
