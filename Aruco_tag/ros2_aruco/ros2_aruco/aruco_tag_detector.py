#!/usr/bin/env python3

"""
This node locates Aruco AR markers in the RGB image, uses depth information
from a registered depth image to compute the 3D position of each marker relative
to the camera, estimates marker orientation, and publishes their poses.

It assumes the depth image is registered (aligned) with the RGB image.

Subscriptions:
   /camera/color/image_raw         (sensor_msgs.msg.Image) - RGB image for Aruco detection
   /camera/aligned_depth_to_color/image_raw (sensor_msgs.msg.Image) - Depth image aligned to color frame
   /camera/color/camera_info       (sensor_msgs.msg.CameraInfo) - Intrinsics for the color camera

Published Topics:
    /aruco_poses              (geometry_msgs.msg.PoseArray) - Array of detected marker poses
    /aruco_markers            (rovers_interfaces.msg.ArucoMarkers) - Marker IDs and poses

Parameters:
    marker_size               Size of the markers edge in meters (default 0.05)
    aruco_dictionary_id       Aruco dictionary name (e.g., DICT_5X5_250) (default DICT_5X5_250)
    camera_frame              Override frame_id for published messages (default uses camera_info frame)
    depth_topic               Topic for the aligned depth image (default /camera/aligned_depth_to_color/image_raw)
    camera_info_topic         Topic for the camera info corresponding to the RGB image (default /camera/color/camera_info)
    image_topic               Topic for the RGB image (default /camera/color/image_raw)

Author: Nathan Sprague (Original)
Modified for Depth Integration and ROS 2: Andreas Bihlmaier, ChatGPT
"""

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from rovers_interfaces.msg import ArucoMarkers  # Ensure this message type exists in your workspace
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys
import traceback  # For detailed error logging

def quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    M = matrix
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
        if M[1, 1] > M[0, 0]:
            i = 1
        if M[2, 2] > M[i, i]:
            i = 2
        nxt = [1, 2, 0]
        j = nxt[i]
        k = nxt[j]
        t = np.sqrt(M[i, i] - M[j, j] - M[k, k] + 1.0)
        q = np.zeros(4)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (M[k, j] - M[j, k]) * t
        q[j] = (M[j, i] + M[i, j]) * t
        q[k] = (M[k, i] + M[i, k]) * t
        x, y, z, w = q[0], q[1], q[2], q[3]
    return [x, y, z, w]

class ArucoDepthNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_depth_node")

        # --- Parameter Declaration ---
        self.declare_parameter(
            name="marker_size",
            value=0.05,  # Default slightly smaller
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers edge in meters.",
            ),
        )
        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary name used for Aruco markers (from cv2.aruco).",
            ),
        )
        self.declare_parameter(
            name="camera_frame",
            value="",  # Default is empty, will use camera_info header
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Override frame_id for published poses. If empty, uses camera_info frame.",
            ),
        )
        self.declare_parameter(
            name="image_topic",
            value="/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic for the RGB image.",
            ),
        )
        self.declare_parameter(
            name="depth_topic",
            value="/camera/aligned_depth_to_color/image_raw",  # Common topic for registered depth
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic for the depth image aligned to the color image.",
            ),
        )
        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/color/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic for the camera info of the RGB camera.",
            ),
        )

        # --- Parameter Retrieval ---
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.camera_frame_override = self.get_parameter("camera_frame").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.get_logger().info(f"Marker size: {self.marker_size}")
        self.get_logger().info(f"Aruco dictionary: {dictionary_id_name}")
        self.get_logger().info(f"Using Image topic: {image_topic}")
        self.get_logger().info(f"Using Depth topic: {depth_topic}")
        self.get_logger().info(f"Using Camera Info topic: {camera_info_topic}")
        if self.camera_frame_override:
            self.get_logger().info(f"Overriding output frame_id with: {self.camera_frame_override}")

        # --- State Variables ---
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.latest_depth_image = None
        self.cv_bridge = None
        self.aruco_dictionary = None
        self.aruco_parameters = None

        # --- Initialization with Error Handling ---
        try:
            # Validate and load Aruco dictionary
            self.get_logger().info(f"Attempting to load Aruco dictionary: {dictionary_id_name}")
            if not hasattr(cv2.aruco, dictionary_id_name):
                raise AttributeError(f"Specified dictionary '{dictionary_id_name}' not found in cv2.aruco.")

            dictionary_id = getattr(cv2.aruco, dictionary_id_name)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)  # Use getPredefinedDictionary
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
            self.get_logger().info("Successfully loaded Aruco dictionary and parameters.")

            # Initialize CvBridge
            self.cv_bridge = CvBridge()
            self.get_logger().info("CvBridge initialized successfully.")

        except AttributeError as e:
            self.get_logger().error(f"Error loading Aruco dictionary: {e}")
            self.get_logger().error("Please ensure the 'aruco_dictionary_id' parameter is a valid name in cv2.aruco (e.g., DICT_5X5_250).")
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(f"Available dictionaries:\n{options}")
            # Prevent the node from continuing without a valid dictionary
            # Consider raising an exception or shutting down cleanly
            self.destroy_node()
            sys.exit(1)  # Exit the script

        except Exception as e:
            self.get_logger().error(f"Failed during initialization: {e}")
            self.get_logger().error(traceback.format_exc())
            self.destroy_node()
            sys.exit(1)  # Exit the script

        # --- ROS Communication Setup ---
        # Quality of Service profile - important for message synchronization
        # Use BEST_EFFORT for potentially lossy sensor streams if synchronization isn't strict
        # Use RELIABLE if messages must be received (might delay processing)
        # History KEEP_LAST with depth 1 is common for grabbing the latest sensor data
        # Adjust based on your OAK-D pipeline's QoS settings
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to Camera Info (using the retrieved topic name)
        # Process this first to get intrinsics
        self.info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.info_callback,
            qos_profile=10  # Use reliable for camera info as it's needed
        )

        # Subscribe to RGB Image (using the retrieved topic name)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos  # Use sensor QoS
        )

        # Subscribe to Depth Image (using the retrieved topic name)
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            sensor_qos  # Use sensor QoS
        )

        # Publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        self.get_logger().info("Aruco Depth Node initialized successfully and waiting for messages.")

    def info_callback(self, info_msg):
        """ Stores camera intrinsics and distortion coefficients. """
        if self.info_msg:  # Check if already processed once
            return  # Avoid redundant processing and logging

        try:
            self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
            self.distortion = np.array(info_msg.d)
            self.info_msg = info_msg  # Store the full message for header info

            # Basic check for valid intrinsics
            if self.intrinsic_mat[0, 0] <= 0 or self.intrinsic_mat[1, 1] <= 0:
                self.get_logger().warn("Received camera info with non-positive focal length (fx or fy).")
                self.intrinsic_mat = None  # Invalidate
                self.info_msg = None
                return

            self.get_logger().info("Received and processed CameraInfo.")
            self.get_logger().info(f"  fx: {self.intrinsic_mat[0, 0]}, fy: {self.intrinsic_mat[1, 1]}")
            self.get_logger().info(f"  cx: {self.intrinsic_mat[0, 2]}, cy: {self.intrinsic_mat[1, 2]}")

        except Exception as e:
            self.get_logger().error(f"Error processing CameraInfo message: {e}")
            self.get_logger().error(traceback.format_exc())
            # Invalidate state if processing failed
            self.info_msg = None
            self.intrinsic_mat = None
            self.distortion = None

    def depth_callback(self, msg):
        """ Converts depth image to a usable format (meters, float32). """
        if not self.cv_bridge:
            self.get_logger().warn("Depth callback called before CvBridge is initialized.")
            return

        # self.get_logger().debug(f"Received depth message with encoding: {msg.encoding}")
        try:
            if msg.encoding == "16UC1":  # Typically millimeters (e.g., RealSense, OAK-D)
                depth_mm = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                # Convert to meters, handling potential NaNs or invalid values if necessary
                # OpenCV operations work best on float types
                self.latest_depth_image = depth_mm.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":  # Typically meters
                self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                self.get_logger().warn(f"Received depth image with unsupported encoding: {msg.encoding}. Attempting passthrough.")
                # Attempt passthrough, but it might fail later if not float/ushort
                self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                # Check type and potentially convert/warn
                if self.latest_depth_image.dtype != np.float32 and self.latest_depth_image.dtype != np.uint16:
                    self.get_logger().error(f"Depth image passthrough resulted in unexpected dtype: {self.latest_depth_image.dtype}. Processing may fail.")
                elif self.latest_depth_image.dtype == np.uint16:
                    self.get_logger().warn("Assuming 16-bit depth is in millimeters. Converting to meters.")
                    self.latest_depth_image = self.latest_depth_image.astype(np.float32) / 1000.0

            # Optional: Add a check for image dimensions if needed
            # if self.latest_depth_image.shape[0] == 0 or self.latest_depth_image.shape[1] == 0:
            #    self.get_logger().warn("Received empty depth image.")
            #    self.latest_depth_image = None

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error converting depth image: {e}")
            self.latest_depth_image = None  # Invalidate on error
        except Exception as e:
            self.get_logger().error(f"Error processing depth message: {e}")
            self.get_logger().error(traceback.format_exc())
            self.latest_depth_image = None  # Invalidate on error

    def image_callback(self, img_msg):
        """ Detects Aruco markers, calculates poses using depth, and publishes. """
        # --- Pre-computation Checks ---
        if not self.info_msg or self.intrinsic_mat is None or self.distortion is None:
            self.get_logger().warn("Waiting for camera info to be processed...", throttle_duration_sec=5)
            return
        if self.latest_depth_image is None:
            self.get_logger().warn("Waiting for depth image...", throttle_duration_sec=5)
            return
        if not self.cv_bridge or not self.aruco_dictionary or not self.aruco_parameters:
            self.get_logger().error("Node not properly initialized (cv_bridge or aruco unavailable). Skipping image processing.")
            return

        # Ensure depth image dimensions match RGB (or handle appropriately if registration is imperfect)
        # This check assumes perfect alignment. Add tolerance if needed.
        if self.latest_depth_image.shape[0] != img_msg.height or self.latest_depth_image.shape[1] != img_msg.width:
            self.get_logger().warn(
                f"Depth image dimensions ({self.latest_depth_image.shape}) "
                f"do not match RGB image dimensions ({img_msg.height}, {img_msg.width}). "
                "Ensure depth image is registered to color frame.",
                throttle_duration_sec=10
            )
            # Option 1: Skip processing this frame
            # return
            # Option 2: Attempt to resize depth (might introduce inaccuracies)
            # try:
            #     depth_resized = cv2.resize(self.latest_depth_image, (img_msg.width, img_msg.height), interpolation=cv2.INTER_NEAREST)
            # except Exception as e:
            #     self.get_logger().error(f"Failed to resize depth image: {e}")
            #     return # Skip if resize fails
            # current_depth_image = depth_resized
            # Option 3: Continue, but depth lookups might be wrong if not aligned (chosen here with warning)
            current_depth_image = self.latest_depth_image  # Use original, assuming registration is intended

        else:
            # Make a local copy to avoid race conditions if depth_callback updates it mid-processing
            current_depth_image = self.latest_depth_image.copy()

        # --- Image Processing and Marker Detection ---
        try:
            # Convert ROS Image message to OpenCV image (grayscale for detection)
            cv_image_rgb = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")  # Use bgr8 for color if needed later
            cv_image_gray = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image_gray, self.aruco_dictionary, parameters=self.aruco_parameters
            )

            # Estimate pose using estimatePoseSingleMarkers for orientation
            # This uses the marker_size parameter and camera intrinsics
            # It gives rotation (rvec) and translation (tvec) relative to camera
            # We will override the translation part with depth data later.
            if marker_ids is not None and len(marker_ids) > 0:
                # Check OpenCV version for return value differences (older versions might not return objPoints)
                if cv2.__version__ >= "4.7.0":
                    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_size, self.intrinsic_mat, self.distortion
                    )
                else:
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_size, self.intrinsic_mat, self.distortion
                    )
            else:
                rvecs = []  # Ensure rvecs is iterable even if no markers detected

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error converting image: {e}")
            return
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error during detection/pose estimation: {e}")
            self.get_logger().error(traceback.format_exc())
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected error during image processing: {e}")
            self.get_logger().error(traceback.format_exc())
            return

        # --- Prepare Output Messages ---
        markers_msg = ArucoMarkers()
        pose_array_msg = PoseArray()

        # Determine the frame_id
        if self.camera_frame_override:
            frame_id = self.camera_frame_override
        elif self.info_msg:
            frame_id = self.info_msg.header.frame_id
        else:
            # Should not happen due to checks above, but as a fallback
            frame_id = "camera_link"  # Or some other default
            self.get_logger().warn(f"Could not determine frame_id, using fallback '{frame_id}'")

        # Set headers
        markers_msg.header.frame_id = frame_id
        pose_array_msg.header.frame_id = frame_id
        markers_msg.header.stamp = img_msg.header.stamp
        pose_array_msg.header.stamp = img_msg.header.stamp

        # --- Process Each Detected Marker ---
        if marker_ids is not None and len(marker_ids) > 0:
            fx = self.intrinsic_mat[0, 0]
            fy = self.intrinsic_mat[1, 1]
            cx = self.intrinsic_mat[0, 2]
            cy = self.intrinsic_mat[1, 2]

            for i, marker_id in enumerate(marker_ids.flatten()):
                marker_corners = corners[i][0]  # Shape (4, 2) -> [[u1,v1], [u2,v2], ...]

                # Calculate the center of the marker in pixel coordinates
                center = np.mean(marker_corners, axis=0)
                center_u = int(round(center[0]))
                center_v = int(round(center[1]))

                # --- Depth Lookup ---
                # Check if the center point is within the depth image bounds
                if not (0 <= center_v < current_depth_image.shape[0] and 0 <= center_u < current_depth_image.shape[1]):
                    self.get_logger().warn(
                        f"Marker ID {marker_id}: Center ({center_u}, {center_v}) is outside depth image bounds "
                        f"({current_depth_image.shape[1]}, {current_depth_image.shape[0]}). Skipping."
                    )
                    continue

                # Get the depth value at the marker's center (in meters)
                depth_value = current_depth_image[center_v, center_u]

                # Validate the depth value
                if depth_value <= 0 or np.isnan(depth_value) or np.isinf(depth_value):
                    self.get_logger().warn(
                        f"Marker ID {marker_id}: Invalid depth value ({depth_value:.3f}m) at center ({center_u}, {center_v}). Skipping."
                    )
                    # Optional: Could try averaging depth over a small ROI around the center
                    continue

                # --- Calculate 3D Position using Pinhole Model ---
                # X = (u - cx) * Z / fx
                # Y = (v - cy) * Z / fy
                # Z = depth_value
                try:
                    X = (center_u - cx) * depth_value / fx
                    Y = (center_v - cy) * depth_value / fy
                    Z = float(depth_value)  # Ensure Z is float

                except ZeroDivisionError:
                    self.get_logger().error("Division by zero error during 3D point calculation (fx or fy is zero). Check camera info.")
                    continue  # Skip this marker

                # --- Create Pose Message ---
                pose = Pose()
                pose.position.x = X
                pose.position.y = Y
                pose.position.z = Z

                # --- Get Orientation from estimatePoseSingleMarkers ---
                try:
                    # Use the rotation vector (rvec) obtained earlier
                    rvec = rvecs[i][0]  # rotation vector for this marker

                    # Convert rotation vector to quaternion
                    # Ensure rvec is a numpy array of shape (3,) or (1, 3) or (3, 1)
                    if not isinstance(rvec, np.ndarray):
                        rvec = np.array(rvec).flatten()

                    # Rodrigues converts rotation vector to rotation matrix
                    rot_matrix, _ = cv2.Rodrigues(rvec)

                    # Convert rotation matrix to quaternion using tf_transformations
                    # Note: tf_transformations expects a 4x4 matrix
                    transform_matrix = np.identity(4)
                    transform_matrix[0:3, 0:3] = rot_matrix
                    quat = quaternion_from_matrix(transform_matrix)

                    pose.orientation.x = float(quat[0])
                    pose.orientation.y = float(quat[1])
                    pose.orientation.z = float(quat[2])
                    pose.orientation.w = float(quat[3])

                except cv2.error as e:
                    self.get_logger().error(f"Marker ID {marker_id}: OpenCV error converting rvec to matrix: {e}. Using default orientation.")
                    # Set default orientation (identity quaternion)
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0
                except Exception as e:
                    self.get_logger().error(f"Marker ID {marker_id}: Error getting orientation: {e}. Using default orientation.")
                    self.get_logger().error(traceback.format_exc())
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0

                # Add the completed pose to the messages
                pose_array_msg.poses.append(pose)
                markers_msg.poses.append(pose)
                markers_msg.marker_ids.append(int(marker_id))  # Ensure marker ID is standard int

            # --- Publish Messages ---
            if len(pose_array_msg.poses) > 0:
                self.poses_pub.publish(pose_array_msg)
                self.markers_pub.publish(markers_msg)
                # self.get_logger().debug(f"Published {len(pose_array_msg.poses)} marker poses.")
        else:
            # Optional: Publish empty messages if no markers are detected
            self.poses_pub.publish(pose_array_msg)
            self.markers_pub.publish(markers_msg)
            # self.get_logger().debug("No markers detected in this frame.")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None  # Initialize to None
    try:
        node = ArucoDepthNode()
        if node and rclpy.ok():  # Check if node initialization was successful before spinning
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e:
        if node:  # Log error using node's logger if available
            node.get_logger().fatal(f"Unhandled exception in main: {e}")
            node.get_logger().fatal(traceback.format_exc())
        else:  # Fallback print if node init failed early
            print(f"Unhandled exception before node logger was fully available: {e}")
            print(traceback.format_exc())
    finally:
        # Cleanup
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
