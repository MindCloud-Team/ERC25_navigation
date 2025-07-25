#!/usr/bin/env python3

"""
This node locates Aruco AR markers in a mono RGB camera image and publishes
their poses relative to the rover using the known marker size for distance estimation.

Subscriptions:
   /oak/rgb/image_raw         (sensor_msgs.msg.Image) - RGB image from mono camera
   /oak/rgb/camera_info       (sensor_msgs.msg.CameraInfo) - Intrinsics for the RGB camera

Published Topics:
    /aruco_poses              (geometry_msgs.msg.PoseArray) - Array of detected marker poses
    /aruco_markers            (rovers_interfaces.msg.ArucoMarkers) - Marker IDs and poses

Parameters:
    marker_size               Size of the markers edge in meters (default 0.05)
    aruco_dictionary_id       Aruco dictionary name (e.g., DICT_ARUCO_ORIGINAL) (default DICT_ARUCO_ORIGINAL)
    rover_frame               Frame ID for the rover (default "base_link")
    image_topic               Topic for the RGB image (default /oak/rgb/image_raw)
    camera_info_topic         Topic for the RGB camera info (default /oak/rgb/camera_info)
"""

import sys
import traceback
import threading
from time import time

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from rovers_interfaces.msg import ArucoMarkers

def quaternion_from_matrix(matrix: np.ndarray) -> list:
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    M = matrix.astype(np.float64)
    t = np.trace(M)
    if t > 0.0:
        t = np.sqrt(t + 1.0)
        w = 0.5 * t
        t = 0.5 / t
        x = (M[2, 1] - M[1, 2]) * t
        y = (M[0, 2] - M[2, 0]) * t
        z = (M[1, 0] - M[0, 1]) * t
    else:
        i = 0
        if M[1, 1] > M[0, 0]: i = 1
        if M[2, 2] > M[i, i]: i = 2
        nxt = [1, 2, 0]
        j = nxt[i]; k = nxt[j]
        t = np.sqrt(M[i, i] - M[j, j] - M[k, k] + 1.0)
        q = [0.0] * 4
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (M[k, j] - M[j, k]) * t
        q[j] = (M[j, i] + M[i, j]) * t
        q[k] = (M[k, i] + M[i, k]) * t
        x, y, z, w = q[0], q[1], q[2], q[3]
    return [float(x), float(y), float(z), float(w)]


class MonoArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("mono_aruco_node")

        # --- Parameter Declaration ---
        self.declare_parameter("marker_size", 0.05, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("rover_frame", "base_link", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("image_topic", "/oak/rgb/image_raw", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        # --- Parameter Retrieval ---
        self.marker_size = self.get_parameter("marker_size").value
        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.rover_frame = self.get_parameter("rover_frame").value
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        # Minimal startup log
        self.get_logger().info(f"Mono ArUco Node: {self.dictionary_id_name}, marker_size={self.marker_size}m")

        # --- State Variables ---
        self.info_msg: CameraInfo | None = None
        self.intrinsic_mat: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_info_received = False
        self.cv_bridge: CvBridge | None = None
        self.aruco_dictionary = None
        self.aruco_parameters = None
        self.aruco_detector = None

        self.info_lock = threading.Lock()

        # OpenCV Version Check
        opencv_version = cv2.__version__.split('.')
        self.opencv_4_7_plus = int(opencv_version[0]) > 4 or (int(opencv_version[0]) == 4 and int(opencv_version[1]) >= 7)

        # --- Initialization ---
        try:
            # Check for required ArUco functions
            if not hasattr(cv2, 'aruco') or \
               not hasattr(cv2.aruco, 'getPredefinedDictionary') or \
               not hasattr(cv2.aruco, 'DetectorParameters') or \
               not hasattr(cv2.aruco, 'ArucoDetector') or \
               not hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
                raise ImportError("Required cv2.aruco functions missing. Ensure 'opencv-contrib-python' is installed correctly.")

            # Initialize ArUco components
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
            self.cv_bridge = CvBridge()

        except ImportError as e:
            self.get_logger().fatal(f"opencv-contrib-python required: {e}")
            sys.exit(1)
        except AttributeError as e:
            self.get_logger().fatal(f"Invalid ArUco dictionary '{self.dictionary_id_name}': {e}")
            sys.exit(1)
        except Exception as e:
            self.get_logger().fatal(f"Init failed: {e}")
            sys.exit(1)

        # --- ROS Communication Setup ---
        self.reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Camera Info Subscriber
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, self.reliable_qos)
        
        # Image Subscriber
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, self.sensor_qos)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

    def info_callback(self, msg: CameraInfo) -> None:
        """Stores camera intrinsics and distortion. Logs success only once."""
        with self.info_lock:
            if self.camera_info_received:
                return
            try:
                if len(msg.k) != 9:
                    return
                k_matrix = np.reshape(np.array(msg.k), (3, 3))
                if k_matrix[0, 0] <= 0 or k_matrix[1, 1] <= 0:
                    return

                self.intrinsic_mat = k_matrix
                self.distortion = np.array(msg.d)
                self.info_msg = msg
                self.camera_info_received = True

            except Exception as e:
                self.get_logger().error(f"Error processing Camera Info: {e}", throttle_duration_sec=10.0)
                self.intrinsic_mat = None
                self.distortion = None
                self.info_msg = None
                self.camera_info_received = False

    def image_callback(self, img_msg: Image):
        """Processes RGB images for ArUco detection and pose estimation."""
        # Quick check without lock first
        if not self.camera_info_received:
            return

        # Get camera parameters (optimized single lock)
        with self.info_lock:
            if not self.camera_info_received:
                return
            local_intrinsic_mat = self.intrinsic_mat
            local_distortion = self.distortion

        # --- Process RGB Image & Detect Markers ---
        try:
            # Convert and detect in one block
            cv_image_rgb = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            cv_image_gray = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2GRAY)
            corners, marker_ids, _ = self.aruco_detector.detectMarkers(cv_image_gray)

            if marker_ids is None or len(marker_ids) == 0:
                # Publish empty messages and return
                empty_markers = ArucoMarkers()
                empty_poses = PoseArray()
                empty_markers.header.stamp = img_msg.header.stamp
                empty_markers.header.frame_id = self.rover_frame
                empty_poses.header.stamp = img_msg.header.stamp  
                empty_poses.header.frame_id = self.rover_frame
                self.poses_pub.publish(empty_poses)
                self.markers_pub.publish(empty_markers)
                return

            # Estimate poses
            if self.opencv_4_7_plus:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, local_intrinsic_mat, local_distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, local_intrinsic_mat, local_distortion)

        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return

        # --- Prepare and Process Markers ---
        markers_msg = ArucoMarkers()
        pose_array_msg = PoseArray()
        
        markers_msg.header.stamp = img_msg.header.stamp
        markers_msg.header.frame_id = self.rover_frame
        pose_array_msg.header.stamp = img_msg.header.stamp
        pose_array_msg.header.frame_id = self.rover_frame

        # Process each detected marker
        for i, marker_id_arr in enumerate(marker_ids):
            if i >= len(rvecs) or i >= len(tvecs):
                continue
                
            marker_id = int(marker_id_arr[0])
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            pose = Pose()
            # Position: transform camera frame to rover frame
            pose.position = Point(
                x=float(tvec[2]),   # Camera Z -> Rover X (forward)
                y=float(-tvec[0]),  # Camera -X -> Rover Y (left)  
                z=float(-tvec[1])   # Camera -Y -> Rover Z (up)
            )

            # Orientation
            if rvec is not None and rvec.shape == (3,) and not np.any(np.isnan(rvec)):
                try:
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    # Transform rotation from camera to rover frame
                    camera_to_rover = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
                    rover_rot_matrix = camera_to_rover @ rot_matrix @ camera_to_rover.T
                    quat = quaternion_from_matrix(rover_rot_matrix)
                    pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                except:
                    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            else:
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            pose_array_msg.poses.append(pose)
            markers_msg.poses.append(pose)
            markers_msg.marker_ids.append(marker_id)

        # Publish results
        self.poses_pub.publish(pose_array_msg)
        self.markers_pub.publish(markers_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MonoArucoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
