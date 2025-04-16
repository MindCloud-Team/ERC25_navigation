#! /usr/bin/env python3
"""
A Sample for Localization of the rover using the OAK-D camera

This sample predicts the position and orientation of the robot
using the OAK-D camera

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

class OAKVisualOdometry(Node):

    """
    This node uses synchronized RGB and stereo depth data for more accurate pose estimation

    This node publishes to /odom and subscribes from (/oak/rgb/camera_info, /oak/rgb/image_rect, /oak/stereo/imgae_raw)

    Attributes:
        odom_pub (Publisher) : publishes estimated odometry data
        bridge (CvBridge) : converts images from ROS formal to opencv format
        orb (ORB) : feature detector
        index_params (dict) : flann parameter
        search_params (dict) : flann parameter
        K (numpy.ndarray) : camera intrinsic matrix
        last_timestamp (builtin_interfaces.msg.Time) : stores time of last processed frame
        self.prev_depth_img (numpy.ndarray) : stores last processed depth image
        self.prev_rgb_img (numpy.ndarray) : stores lasst processed RGB image
        C_k (numpy.ndarray) : current transformation matrix
        estimates (list) : estimated odom

    ROS Publishers:
        /odom (nav_msgs/Odometry)

    ROS Subscribers:
        /oak/rgb/camera_info (sensormsgs/CameraInfo)
        /oak/rgb/image_rect (sensormsgs/Image)
        /oak/stereo/image_raw (sensormsgs/Image)

    """

    def __init__(self):
        """
        Initializes the Visual Odometry.

        Sets up the ROS publishers, subscribers.
        """
        super().__init__("oak_visual_odom")

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize ORB detector
        self.orb = cv2.ORB.create(3000)
        self.index_params = dict(algorithm=6, trees=5)
        self.search_params = dict(checks=50)

        # Initialize attributes
        self.K = None  # Camera intrinsic matrix
        self.C_k = np.eye(4)  # Initial transformation matrix
        self.last_timestamp = None
        self.prev_rgb_img = None
        self.prev_depth_img = None
        self.estimates = []

        # Subscribe to camera calibration info first
        self.calib_sub = self.create_subscription(
            CameraInfo,
            "/oak/rgb/camera_info",
            self.camera_info_callback,
            10
        )

        self.get_logger().info("OAK Visual Odometry node initialized")

    def camera_info_callback(self, msg):
        """
        Process camera calibration information
        """
        if self.K is None:  # Only set once
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera calibration received")

            # Now that we have camera calibration, set up synchronized subscribers
            self.setup_synchronized_subscribers()

            # Unsubscribe from the calibration topic
            self.destroy_subscription(self.calib_sub)

    def setup_synchronized_subscribers(self):
        """
        Set up synchronized subscribers for RGB and depth images
        """
        self.rgb_sub = message_filters.Subscriber(self, Image, '/oak/rgb/image_rect')
        self.depth_sub = message_filters.Subscriber(self, Image, '/oak/stereo/image_raw')

        # Synchronize the messages with a time tolerance of 0.1 seconds
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.get_logger().info("Synchronized subscribers set up")

    def synchronized_callback(self, rgb_msg, depth_msg):
        """
        Process synchronized RGB and depth images
        """
        try:
            # Convert ROS messages to OpenCV images with correct encoding
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "mono8")

            # For depth image, use the actual encoding from the message
            depth_encoding = depth_msg.encoding
            self.get_logger().info(f"Depth image encoding: {depth_encoding}")

            # Convert depth image properly (16UC1 for depth)
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_encoding)

            # Process frames if we have previous frames
            if self.prev_rgb_img is not None and self.prev_depth_img is not None:
                self.process_frames(
                    self.prev_rgb_img,
                    rgb_img,
                    self.prev_depth_img,
                    depth_img,
                    rgb_msg.header.stamp
                )

            # Store current frames for next iteration
            self.prev_rgb_img = rgb_img
            self.prev_depth_img = depth_img
            self.last_timestamp = rgb_msg.header.stamp

        except Exception as e:
            self.get_logger().error(f"Error in synchronized callback: {str(e)}")

    def process_frames(self, prev_rgb, curr_rgb, prev_depth, curr_depth, timestamp):
        """
        Processes subsequent RGB frames and publishes to /odom
        """
        try:
            pts1, pts2, good_matches = self.match_features(prev_rgb, curr_rgb)
            if pts1 is None or len(pts1) < 8:
                self.get_logger().warn("Not enough good matches found")
                return

            pts3d, valid_indices = self.get_3d_points(pts1, prev_depth)
            if pts3d is None or len(pts3d) < 4:
                self.get_logger().warn("Not enough valid 3D points")
                return

            # Filter the 2D points to only include those with valid depth
            pts2_valid = pts2[valid_indices]

            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                pts3d, pts2_valid, self.K, None, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success or inliers is None or len(inliers) < 5:
                self.get_logger().warn(f"PnP failed or too few inliers: {0 if inliers is None else len(inliers)}")
                return

            # Rest of your code for updating pose...
            R_mat, _ = cv2.Rodrigues(rvec)
            T_k = np.eye(4)
            T_k[:3, :3] = R_mat
            T_k[:3, 3] = tvec.flatten()
            self.C_k = self.C_k @ T_k

            odom_msg = self.create_odom_message(self.C_k, timestamp)
            self.odom_pub.publish(odom_msg)
            self.estimates.append(self.C_k.copy())
            self.get_logger().info(f"New pose estimated with {len(inliers)} inliers")

        except Exception as e:
            self.get_logger().error(f"Error processing frames: {str(e)}")

    def match_features(self, img1, img2):
        """
        Feature detection and matching between frames
        """
        # Calculate keypoints and descriptors
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)

        if des1 is None or des2 is None or len(des1) < 2 or len(des2) < 2:
            self.get_logger().warn("Not enough features detected")
            return None, None, None

        # Match features using FLANN
        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Filter good matches using Lowe's ratio test
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

        # Extract matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])

        return pts1, pts2, good_matches

    def get_3d_points(self, pts2d, depth_img):
        """
        Convert 2D points to 3D using depth image, and return valid indices.
        """
        pts3d = []
        valid_indices = []
        depth_type = depth_img.dtype
        self.get_logger().info(f"Depth image type: {depth_type}, shape: {depth_img.shape}")

        scale_factor = 1000.0 if depth_type == np.uint16 else 1.0

        for i, (u, v) in enumerate(pts2d):
            u_int, v_int = int(round(u)), int(round(v))
            if 0 <= u_int < depth_img.shape[1] and 0 <= v_int < depth_img.shape[0]:
                depth_value = float(depth_img[v_int, u_int]) / scale_factor
                if depth_value <= 0.1 or depth_value > 10.0:
                    continue
                x = (u - self.K[0, 2]) * depth_value / self.K[0, 0]
                y = (v - self.K[1, 2]) * depth_value / self.K[1, 1]
                z = depth_value
                pts3d.append([x, y, z])
                valid_indices.append(i)

        if len(pts3d) < 4:
            self.get_logger().warn(f"Only {len(pts3d)} valid 3D points found")
            return None, None

        return np.array(pts3d, dtype=np.float32), valid_indices

    def create_odom_message(self, T, timestamp):
        """
        Create an odometry message from transformation matrix
        """
        # Extract translation and rotation
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position and orientation
        odom.pose.pose.position = Point(
            x=translation[0],
            y=translation[1],
            z=translation[2]
        )
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # Calculate velocity if we have previous estimates
        if self.last_timestamp is not None and len(self.estimates) > 1:
            dt = (timestamp.sec - self.last_timestamp.sec) + \
                 (timestamp.nanosec - self.last_timestamp.nanosec) * 1e-9

            if dt > 0.001:  # Ensure we have a meaningful time difference
                prev_T = self.estimates[-2]
                prev_translation = prev_T[:3, 3]

                # Linear velocity
                velocity = (translation - prev_translation) / dt
                odom.twist.twist.linear.x = float(velocity[0])
                odom.twist.twist.linear.y = float(velocity[1])
                odom.twist.twist.linear.z = float(velocity[2])

                # Angular velocity calculation (simplified)
                prev_rotation = prev_T[:3, :3]
                rel_rotation = rotation_matrix @ prev_rotation.T
                rotvec = R.from_matrix(rel_rotation).as_rotvec()
                angular_velocity = rotvec / dt

                odom.twist.twist.angular.x = float(angular_velocity[0])
                odom.twist.twist.angular.y = float(angular_velocity[1])
                odom.twist.twist.angular.z = float(angular_velocity[2])

        return odom

    def visualize_trajectory(self):
        """
        Visualize the estimated trajectory
        """
        if len(self.estimates) < 2:
            self.get_logger().warn("Not enough poses to visualize trajectory")
            return

        # Extract trajectory
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


def main(args=None):
    """
    Main entry point for Visual Odometry node
    """
    rclpy.init(args=args)
    node = OAKVisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Visualize trajectory before shutdown
        node.visualize_trajectory()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
