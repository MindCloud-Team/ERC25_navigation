"""
This node locates Aruco AR markers in the RGB image, uses depth information from the stereo camera 
to compute the 3D position of each marker relative to the camera, and publishes their poses.

Subscriptions:
   /oak/rgb/image_raw         (sensor_msgs.msg.Image) - for Aruco detection
   /oak/stereo/camera_info    (sensor_msgs.msg.CameraInfo) - for stereo camera intrinsics
   /stereo/depth              (sensor_msgs.msg.Image) - depth image (depth in meters or converted to meters)

Published Topics:
    /aruco_poses              (geometry_msgs.msg.PoseArray)
    /aruco_markers            (ros2_aruco_interfaces.msg.ArucoMarkers)

Parameters:
    marker_size               Size of the markers in meters (default 0.0625)
    aruco_dictionary_id       Aruco dictionary used for detection (default DICT_5X5_250)
    camera_frame              Camera frame to assign to published poses (if empty, use camera info frame)

Author: Nathan Sprague (modified for stereo depth integration)
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArucoDepthNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_depth_node")

        # Subscribe to stereo camera info (intrinsics for depth calculation)
        self.info_sub = self.create_subscription(
            CameraInfo, "/oak/stereo/camera_info", self.info_callback, qos_profile=10
        )

        # Subscribe to the RGB image for Aruco detection
        self.image_sub = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, qos_profile=10
        )

        # Subscribe to the depth image (assumed aligned with RGB image)
        self.depth_sub = self.create_subscription(
            Image, "/oak/stereo/image_raw", self.depth_callback, qos_profile=10
        )

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )
        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary used for Aruco markers.",
            ),
        )
        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame to assign to published poses.",
            ),
        )

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Validate and obtain the Aruco dictionary
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("Invalid aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("Valid options: {}".format(options))
            return

        # Set up publishers for marker poses
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Fields to store camera parameters and latest depth image
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.depth_image = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)
        self.get_logger().info("Received stereo camera info.")

    def depth_callback(self, msg):
        # Convert depth image using CvBridge. Adjust depending on encoding.
        try:
            if msg.encoding == "16UC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.depth_image = depth.astype(np.float32) / 1000.0  # convert from mm to meters
            elif msg.encoding == "32FC1":
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                self.get_logger().warn("Unsupported depth encoding: {}".format(msg.encoding))
        except Exception as e:
            self.get_logger().error("Error converting depth image: {}".format(e))

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info received yet!")
            return
        if self.depth_image is None:
            self.get_logger().warn("No depth image received yet!")
            return

        # Convert the RGB image to grayscale for marker detection.
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        # Prepare header info for published messages.
        markers = ArucoMarkers()
        pose_array = PoseArray()
        frame_id = self.camera_frame if self.camera_frame != "" else self.info_msg.header.frame_id
        markers.header.frame_id = frame_id
        pose_array.header.frame_id = frame_id
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        if marker_ids is not None:
            # Obtain rotation vectors for orientation (if needed)
            if cv2.__version__ > "4.0.0":
                rvecs, _, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            else:
                rvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)

            for i, marker_id in enumerate(marker_ids):
                # Compute the center of the marker (average of its corners)
                marker_corners = corners[i][0]  # shape (4,2)
                center = np.mean(marker_corners, axis=0)
                center_u = int(round(center[0]))
                center_v = int(round(center[1]))

                # Check bounds on the depth image
                if center_v < 0 or center_v >= self.depth_image.shape[0] or center_u < 0 or center_u >= self.depth_image.shape[1]:
                    self.get_logger().warn("Marker center out of depth image bounds")
                    continue

                # Lookup the depth at the center pixel
                depth_value = self.depth_image[center_v, center_u]
                if depth_value <= 0:
                    self.get_logger().warn("Invalid depth at marker center")
                    continue

                # Compute 3D coordinates using the pinhole camera model:
                # X = (u - cx)*Z/fx, Y = (v - cy)*Z/fy, Z = depth value.
                cx = self.intrinsic_mat[0, 2]
                cy = self.intrinsic_mat[1, 2]
                fx = self.intrinsic_mat[0, 0]
                fy = self.intrinsic_mat[1, 1]
                X = (center_u - cx) * depth_value / fx
                Y = (center_v - cy) * depth_value / fy
                Z = depth_value

                pose = Pose()
                pose.position.x = float(X)
                pose.position.y = float(Y)
                pose.position.z = float(Z)

                # For orientation, use the rotation vector from Aruco detection.
                # (This orientation is computed using the known marker size and the image projection.)
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)
                pose.orientation.x = float(quat[0])
                pose.orientation.y = float(quat[1])
                pose.orientation.z = float(quat[2])
                pose.orientation.w = float(quat[3])

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

def main():
    rclpy.init()
    node = ArucoDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
