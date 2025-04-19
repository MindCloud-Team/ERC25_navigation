#!/usr/bin/env python3

"""
This node locates Aruco AR markers in the RGB image, uses depth information
from the UNALIGNED stereo depth image (16UC1 format expected, converted to meters)
to compute the 3D position of each marker relative to the STEREO camera,
and publishes their poses.

Uses message_filters for robust synchronization between RGB and Depth images.
Designed for OpenCV >= 4.7.0 (tested with 4.12.0-dev). Requires opencv-contrib-python.

Subscriptions:
   /oak/rgb/image_raw         (sensor_msgs.msg.Image) - RGB image (synchronized)
   /oak/stereo/image_raw      (sensor_msgs.msg.Image) - UNALIGNED Depth image (synchronized, expects 16UC1, mm)
   /oak/stereo/camera_info    (sensor_msgs.msg.CameraInfo) - Intrinsics for the STEREO camera (separate sub)

Published Topics:
    /aruco_poses              (geometry_msgs.msg.PoseArray) - Array of detected marker poses
    /aruco_markers            (rovers_interfaces.msg.ArucoMarkers) - Marker IDs and poses

Parameters:
    marker_size               Size of the markers edge in meters (default 0.05)
    aruco_dictionary_id       Aruco dictionary name (e.g., DICT_ARUCO_ORIGINAL) (default DICT_ARUCO_ORIGINAL)
    camera_frame              Override frame_id for published messages (default uses STEREO camera_info frame)
    image_topic               Topic for the RGB image (default /oak/rgb/image_raw)
    depth_topic               Topic for the UNALIGNED depth image (default /oak/stereo/image_raw)
    camera_info_topic         Topic for the STEREO camera info (default /oak/stereo/camera_info)
    depth_window_size         Size of window for depth averaging (default 3)
    approx_sync_slop_sec      Maximum time difference (seconds) for approximate sync (default 0.05 -> 50ms)
"""

import sys
import traceback
import threading
from time import time
import collections

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from rovers_interfaces.msg import ArucoMarkers

def quaternion_from_matrix(matrix: np.ndarray) -> list:
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    # (Implementation remains the same)
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


class ArucoDepthNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_depth_node")

        # --- Parameter Declaration ---
        self.declare_parameter("marker_size", 0.05, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("camera_frame", "", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("image_topic", "/oak/rgb/image_raw", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("depth_topic", "/oak/stereo/image_raw", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("camera_info_topic", "/oak/stereo/camera_info", ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter("depth_window_size", 3, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter("approx_sync_slop_sec", 0.05, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))

        # --- Parameter Retrieval ---
        self.marker_size = self.get_parameter("marker_size").value
        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.camera_frame_override = self.get_parameter("camera_frame").value
        self.image_topic = self.get_parameter("image_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_window_size = self.get_parameter("depth_window_size").value
        self.approx_sync_slop_sec = self.get_parameter("approx_sync_slop_sec").value

        # Validate depth window size
        if self.depth_window_size <= 0: self.depth_window_size = 1
        elif self.depth_window_size % 2 == 0: self.depth_window_size += 1

        # --- Log Configuration ---
        self.get_logger().info("--- Aruco Depth Node Configuration (Unaligned Depth | Approx Time Sync) ---")
        self.get_logger().warn("*** Using UNALIGNED depth (16UC1) & ApproximateTimeSynchronizer ***")
        self.get_logger().info(f"Marker size: {self.marker_size}, Dictionary: {self.dictionary_id_name}")
        self.get_logger().info(f"Syncing Topics: RGB='{self.image_topic}', Depth='{self.depth_topic}'")
        self.get_logger().info(f"Stereo Info Topic: {self.camera_info_topic}")
        self.get_logger().info(f"Depth Window: {self.depth_window_size}, Sync Slop: {self.approx_sync_slop_sec:.3f}s")
        log_frame = self.camera_frame_override if self.camera_frame_override else "from STEREO Camera Info"
        self.get_logger().info(f"Output frame_id: {log_frame}")
        self.get_logger().info("--- End Configuration ---")

        # --- State Variables ---
        self.info_msg: CameraInfo | None = None
        self.intrinsic_mat: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_info_received = False # Flag to log info receipt only once
        self.cv_bridge: CvBridge | None = None
        self.aruco_dictionary = None
        self.aruco_parameters = None
        self.aruco_detector = None

        self.info_lock = threading.Lock()

        # OpenCV Version Check (for estimatePoseSingleMarkers return values)
        self.opencv_version = cv2.__version__.split('.')
        self.opencv_4_7_plus = int(self.opencv_version[0]) > 4 or (int(self.opencv_version[0]) == 4 and int(self.opencv_version[1]) >= 7)
        self.get_logger().info(f"OpenCV Version: {cv2.__version__} (Contrib Required!)")

        # --- Initialization ---
        try:
            # *** CRITICAL CHECK for required functions ***
            if not hasattr(cv2, 'aruco') or \
               not hasattr(cv2.aruco, 'getPredefinedDictionary') or \
               not hasattr(cv2.aruco, 'DetectorParameters') or \
               not hasattr(cv2.aruco, 'ArucoDetector') or \
               not hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
                raise ImportError("Required cv2.aruco functions missing. Ensure 'opencv-contrib-python' is installed correctly in your ROS environment.")

            self.get_logger().info(f"Loading Aruco dictionary: {self.dictionary_id_name}")
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
            self.get_logger().info("Aruco detector initialized successfully.")

            self.cv_bridge = CvBridge()
            self.get_logger().info("CvBridge initialized successfully.")

        except ImportError as e:
             self.get_logger().fatal(f"Initialization failed: {e}")
             self.get_logger().fatal("Please verify your 'opencv-contrib-python' installation.")
             self.destroy_node(); sys.exit(1)
        except AttributeError as e:
            self.get_logger().fatal(f"Error accessing Aruco dictionary '{self.dictionary_id_name}'. Is it valid? {e}")
            self.destroy_node(); sys.exit(1)
        except Exception as e:
            self.get_logger().fatal(f"Failed during initialization: {e}\n{traceback.format_exc()}")
            self.destroy_node(); sys.exit(1)

        # --- ROS Communication Setup ---
        self.reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1) # Depth 1 for camera info
        self.sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Camera Info Subscriber
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, self.reliable_qos)

        # Message Filters Subscribers
        self.image_sub = Subscriber(self, Image, self.image_topic, qos_profile=self.sensor_qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=self.sensor_qos)

        # Approximate Time Synchronizer
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=10, slop=self.approx_sync_slop_sec
        )
        self.time_synchronizer.registerCallback(self.synchronized_callback)

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        self.get_logger().info("Node initialized. Waiting for STEREO camera info and synchronized messages...")


    def info_callback(self, msg: CameraInfo) -> None:
        """ Stores STEREO camera intrinsics and distortion. Logs success only once. """
        with self.info_lock:
            if self.camera_info_received: # Already processed successfully
                return
            try:
                if len(msg.k) != 9: return # Invalid K
                k_matrix = np.reshape(np.array(msg.k), (3, 3))
                if k_matrix[0, 0] <= 0 or k_matrix[1, 1] <= 0: return # Invalid focal lengths

                self.intrinsic_mat = k_matrix
                self.distortion = np.array(msg.d)
                self.info_msg = msg
                self.camera_info_received = True # Set flag AFTER successful processing

                self.get_logger().info(f"Successfully received STEREO CameraInfo (fx={k_matrix[0,0]:.1f}, fy={k_matrix[1,1]:.1f}). Ready.")

            except Exception as e:
                # Log error if processing fails, but allow retries on next message
                self.get_logger().error(f"Error processing STEREO CameraInfo: {e}", throttle_duration_sec=10.0)
                # Reset state if error occurs during processing
                self.intrinsic_mat = None; self.distortion = None; self.info_msg = None
                self.camera_info_received = False # Ensure we try again

    def get_windowed_depth(self, depth_image: np.ndarray, center_u: int, center_v: int, window_size: int) -> float:
        """ Get median depth in a window around the pixel (u,v), ignoring zeros. """
        # (Implementation remains the same)
        if window_size <= 0: return 0.0
        half_size = window_size // 2
        height, width = depth_image.shape
        u_min = max(0, center_u - half_size)
        u_max = min(width, center_u + half_size + 1)
        v_min = max(0, center_v - half_size)
        v_max = min(height, center_v + half_size + 1)
        if v_min >= v_max or u_min >= u_max: return 0.0
        window = depth_image[v_min:v_max, u_min:u_max]
        valid_depths = window[window > 0]
        return float(np.median(valid_depths)) if valid_depths.size > 0 else 0.0

    def synchronized_callback(self, img_msg: Image, depth_msg: Image):
        """ Processes synchronized RGB and Depth images for Aruco detection and pose estimation. """
        # Quick check if camera info is ready (avoids locking if not needed)
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for STEREO camera info before processing synchronized messages...", throttle_duration_sec=5.0)
            return

        # Acquire camera info once at the beginning of the callback
        with self.info_lock:
            # Check again inside lock in case it changed between outer check and lock acquisition
            if not self.camera_info_received or self.intrinsic_mat is None or self.distortion is None:
                 self.get_logger().warn("Camera info lost or invalid after lock. Skipping frame.", throttle_duration_sec=5.0)
                 return
            local_intrinsic_mat = self.intrinsic_mat.copy()
            local_distortion = self.distortion.copy()
            # Get frame_id safely, handle None case
            frame_id = self.camera_frame_override or (self.info_msg.header.frame_id if self.info_msg else None) or "stereo_camera_link"
            cam_height = self.info_msg.height if self.info_msg else 0
            cam_width = self.info_msg.width if self.info_msg else 0


        # Check essential components
        if not self.cv_bridge or not self.aruco_detector:
            self.get_logger().error("Node components (cv_bridge/aruco_detector) not ready. Skipping.")
            return

        # --- Process Depth Image ---
        try:
            local_depth_image_meters = None
            if depth_msg.encoding == "16UC1":
                depth_mm = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                local_depth_image_meters = depth_mm.astype(np.float32) / 1000.0
            elif depth_msg.encoding == "32FC1":
                local_depth_image_meters = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            else:
                self.get_logger().warn(f"Depth image encoding {depth_msg.encoding} unsupported. Skipping.")
                return
            invalid_mask = np.logical_or(local_depth_image_meters <= 0, np.isnan(local_depth_image_meters), np.isinf(local_depth_image_meters))
            local_depth_image_meters[invalid_mask] = 0.0
        except CvBridgeError as e: self.get_logger().error(f"CvBridge Error (Depth): {e}"); return
        except Exception as e: self.get_logger().error(f"Error processing depth msg: {e}\n{traceback.format_exc()}"); return

        # --- Process RGB Image & Detect Markers ---
        rvecs = []; tvecs = []; marker_ids = None; corners = None; markers_found = 0
        try:
            cv_image_rgb = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            # Optional Dimension Check (using info acquired under lock)
            if cam_height > 0 and cam_width > 0 and \
               (cv_image_rgb.shape[0] != cam_height or cv_image_rgb.shape[1] != cam_width):
                 self.get_logger().warn(f"RGB dims mismatch CamInfo.", throttle_duration_sec=10.0)

            cv_image_gray = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2GRAY)
            corners, marker_ids, rejected = self.aruco_detector.detectMarkers(cv_image_gray)

            # --- Estimate Pose ---
            if marker_ids is not None and len(marker_ids) > 0:
                markers_found = len(marker_ids)
                # *** Call the function directly now that we check it exists during init ***
                if self.opencv_4_7_plus:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, local_intrinsic_mat, local_distortion)
                else: # Fallback for older OpenCV (less likely now)
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, local_intrinsic_mat, local_distortion)

                if len(rvecs) != markers_found or len(tvecs) != markers_found:
                    self.get_logger().error(f"Marker/Pose count mismatch ({markers_found} vs {len(rvecs)}/{len(tvecs)}).")
                    markers_found = 0; rvecs = []; tvecs = [] # Reset

        except CvBridgeError as e: self.get_logger().error(f"CvBridge Error (RGB): {e}"); return
        except cv2.error as e: self.get_logger().error(f"OpenCV error during detection/pose: {e}"); return # Less detail needed now
        except Exception as e: self.get_logger().error(f"Unexpected error during detection/pose: {e}"); return # Less detail needed now

        # --- Prepare Output Messages ---
        markers_msg = ArucoMarkers()
        pose_array_msg = PoseArray()
        current_time = img_msg.header.stamp # Use synchronized image timestamp
        markers_msg.header.stamp = current_time
        markers_msg.header.frame_id = frame_id
        pose_array_msg.header.stamp = current_time
        pose_array_msg.header.frame_id = frame_id

        # --- Process Each Detected Marker ---
        num_poses_calculated = 0
        if markers_found > 0 and marker_ids is not None:
            fx = local_intrinsic_mat[0, 0]; fy = local_intrinsic_mat[1, 1]
            cx = local_intrinsic_mat[0, 2]; cy = local_intrinsic_mat[1, 2]

            # Check intrinsics needed for calculation (already checked fx,fy>0 in info_callback)
            for i, marker_id_arr in enumerate(marker_ids):
                marker_id = int(marker_id_arr[0])
                try:
                    if i >= len(corners) or i >= len(rvecs) or i >= len(tvecs): continue # Skip if index OOB

                    marker_corners = corners[i][0]
                    center_pixel = np.mean(marker_corners, axis=0)
                    center_u, center_v = int(round(center_pixel[0])), int(round(center_pixel[1]))

                    depth_m = self.get_windowed_depth(local_depth_image_meters, center_u, center_v, self.depth_window_size)
                    if depth_m <= 0: continue # Skip if no valid depth

                    X = (center_u - cx) * depth_m / fx
                    Y = (center_v - cy) * depth_m / fy
                    Z = float(depth_m)

                    pose = Pose()
                    pose.position = Point(x=float(X), y=float(Y), z=Z)

                    rvec = rvecs[i][0]
                    if rvec is not None and isinstance(rvec, np.ndarray) and rvec.shape == (3,) and not np.any(np.isnan(rvec)) and not np.any(np.isinf(rvec)):
                        try:
                            rot_matrix, _ = cv2.Rodrigues(rvec)
                            quat = quaternion_from_matrix(rot_matrix)
                            pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                        except Exception as e_rot:
                            self.get_logger().warn(f"Marker {marker_id}: Orientation error {e_rot}. Using default.")
                            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    else:
                         self.get_logger().warn(f"Marker {marker_id}: Invalid rvec. Using default orientation.")
                         pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                    pose_array_msg.poses.append(pose)
                    markers_msg.poses.append(pose)
                    markers_msg.marker_ids.append(marker_id)
                    num_poses_calculated += 1

                except Exception as e_marker_loop:
                    self.get_logger().warn(f"Error processing marker {marker_id}: {e_marker_loop}")
                    continue # Skip to next marker

        # --- Publish Messages ---
        # No need for verbose logs here unless debugging is needed
        self.poses_pub.publish(pose_array_msg)
        self.markers_pub.publish(markers_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDepthNode()
        rclpy.spin(node) # Handles exceptions during spin internally mostly
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e: # Catch unexpected errors during init or shutdown
        if node: node.get_logger().fatal(f"Unhandled exception in main: {e}\n{traceback.format_exc()}")
        else: print(f"Unhandled exception before/during node init: {e}\n{traceback.format_exc()}")
    finally:
        if node and rclpy.ok():
             # This ensures node resources are cleaned up if shutdown wasn't clean
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
