#################                        STEREO VISUAL ODOMETRY CODE                        ###############

#                                   Testing was done using the KITTI dataset.

import rclpy
import cv2
import os
import matplotlib.pyplot as plt
import scipy.optimize as opt
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
import time
class VisualOdometry(Node):

    """
    ROS2 node for locating the rover using visual odometry
    
    This node publishes to /odom

    Attributes:
        pub (Publisher): publishes estimated odometry data
        accurate_pub (Publisher): publishes true odometry data
        imgs_l (list) : paths of left camera images
        imgs_r (list) : paths of right camera images
        orb (ORB) : feature detector
        index_params (dict) : flann parameter
        search_params (dict) : flann parameter
        K_l, P_l, K_r, P_r (all ndarray) : intrinsic coordinates of the cameras 
        disparity : depth map
        C_k (ndarray) : current transformation matrix
        estimates (list) : estimated odom
        timestamps (list) : times of estimated odom calculation
        timestamps_truth (list) : times of true odom calculation
        poses (list) : loaded poses provided by KITTIimgs_l : paths of left camera images

    ROS Publishers:
        /odom (nav_msgs/Odometry)
    """
    def __init__(self):
        
        """
        Initializes the Visual Odometry.

        Sets up the ROS publishers.
        """

        super().__init__("visual_odom")
        self.pub  = self.create_publisher(Odometry, "/odom", 10)
        self.accurate_pub = self.create_publisher(Odometry, "/odompro", 10)
        self.lsub = self.create_subscription(Image, "left/image_rect", callback= self.left_handle, qos_profile=10)
        self.depth_sub = self.create_subscription(Image, "stereo/depth", callback = self.depth_handle, qos_profile = 10)
        # Extracting images from the training folders

        #self.imgs_l = self.load_images("/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/image_2")
        self.imgs_r = self.load_images("/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/image_3")
        # Initializing ORB and FLANN parameters
        
        self.orb = cv2.ORB.create(4000)
        self.index_params = dict(algorithm=6, trees=5)
        self.search_params = dict(checks=50)
        
        filepath = "/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/calib.txt"
        self.K_l, self.P_l, self.K_r, self.P_r = self.load_calibration(filepath)
        
        # Generating the depth map

        # May be needed if we're dividing the image if we use the FAST detector

        block = 11
        P1 = block * block * 8
        P2 = block * block * 32
        self.disparity = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=32, blockSize=block, P1=P1, P2=P2
        )

        # Initializing Attributes

        self.C_k = np.eye(4) 
        self.estimates = []
        self.timestamps = []
        self.timestamps_truth = []

        self.poses = self.load_poses("/home/ibrahim/ws/src/visual_odom/kitti_poses/00.txt") # Loading the ground truth poses 
        
        self.process_sequence() # running the pose odom publisher 
        
        self.visualizer() # Visualizing estimated odom with ground truth odom

    def load_images(self, filepath):

        """
        Loads the images
        """

        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]
        images = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]
        return images

    def matching(self, img1, img2):
        """
        Feature detction and matching
        """
        # Calculating Keypoints of the features in both frames
        
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)

        # Matching using FLANN

        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Storing the good matches

        good_matches = [m for m, n in matches if m.distance < 0.8 * n.distance]
        pts1, pts2 = self.extract_matched_points(kp1, kp2, good_matches)
        return pts1, pts2

    def extract_matched_points(self, kp1, kp2, matches):
        """
        Extracting points from keypoints
        """
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        return pts1, pts2

    def calc_essential_matrix(self, pts1, pts2):
        
        """
        Calculates the Essential Matrix
        """

        E, mask = cv2.findEssentialMat(pts1, pts2, self.K_l, method=cv2.RANSAC, prob=0.999, threshold=0.5)

        # Decomposing the Essential Matrix

        _, R, T, mask = cv2.recoverPose(E, pts1, pts2, self.K_l)
    
        T_mat = np.eye(4)

        # Forming the transformation matrix

        T_mat[:3, :3] = R
        T_mat[:3, 3] = T.flatten()
        return T_mat

    def triangulate_points(self, pts1, pts2):
        
        """
        Performs 3D triangulation
        May be useful in the future
        """

        pts1_norm = cv2.undistortPoints(pts1, self.K_l, None)
        pts2_norm = cv2.undistortPoints(pts2, self.K_r, None)

        # Solving an incorrect shape error

        pts1_norm = pts1_norm.reshape(-1, 2).T  
        pts2_norm = pts2_norm.reshape(-1, 2).T  
        print("pts1_norm shape:", pts1_norm.shape)  
        print("pts2_norm shape:", pts2_norm.shape)  
        print("P_l:", self.P_l)
        print("P_r:", self.P_r)

        if np.isnan(pts1_norm).any() or np.isnan(pts2_norm).any():
            print("NaN values detected in normalized points!")
            return
        if np.isinf(pts1_norm).any() or np.isinf(pts2_norm).any():
            print("Infinite values detected in normalized points!")
            return

        points_4d = cv2.triangulatePoints(self.P_l, self.P_r, pts1_norm.T, pts2_norm.T)
        points_3d = points_4d[:3, :] / points_4d[3, :]
        return points_3d.T

    def load_calibration(self, filepath):
        
        """
        Extracts calibration data from the file provided
        """

        with open(filepath, 'r') as f:
            lines = f.readlines()
            lines = [line.strip().split(":")[1] for line in lines if ":" in line]  # Removing labels

        P_l = np.reshape(np.fromstring(lines[0], dtype=np.float64, sep=' '), (3, 4))
        P_r = np.reshape(np.fromstring(lines[1], dtype=np.float64, sep=' '), (3, 4))
        K_l = P_l[:, :3]
        K_r = P_r[:, :3]
        return K_l, P_l, K_r, P_r
    
    def load_poses(self,filepath):
        
       """
       Extracts true pose data from the provided text file
       """

       poses = []
       with open(filepath, 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, dtype=np.float64, sep=' ')
                T = T.reshape(3, 4)
                T = np.vstack((T, [0, 0, 0, 1]))
                poses.append(T)
       return poses
        
    def odom_handle(self, T,i):
        
        """
        Extracts position and orientation from the matrix
        Must be done to publish to /odom
        """
        
        # Setting the position and orientation
        
        odom, rotation_matrix, translation, quaternion = self.odom_shorten(T)

        odom.pose.pose.position = Point(
            x = translation[0], y = translation[1], z = translation[2]*-1)
        odom.pose.pose.orientation = Quaternion(
            x = quaternion[0], y = quaternion[1], z = quaternion[2], w =quaternion[3]
        )   
        
        current_time = self.get_clock().now()
        self.timestamps.append(current_time)

        # Compute linear velocity

        if len(self.timestamps) > 1:
            dt = (self.timestamps[-1] - self.timestamps[-2]).nanoseconds * 1e-9

            if dt == 0:  # Preventing division by zero
                dt = 1e-6
        else:
            dt = 0.1  # Initializing
        if len(self.estimates) > 1:
            prev_T = self.estimates[i-1]  # Previous transformation matrix
            prev_translation = prev_T[:3, 3]  # Previous position
            prev_rotation_matrix = prev_T[:3, :3]  # Previous rotation
            rot_diff = rotation_matrix @ prev_rotation_matrix.T  # Relative rotation
            U, _, Vt = np.linalg.svd(rot_diff)
            rot_diff = U @ Vt  # Ensure it is a valid rotation matrix
            angular_velocity = R.from_matrix(rot_diff).as_rotvec() / dt

        else:

            # Inititializing default values

            prev_translation = np.zeros(3)
            prev_rotation_matrix = np.eye(3) 
            angular_velocity = np.zeros(shape = (3,3))

        # Computing velocity values

        velocity_x = (translation[0] - prev_translation[0])/dt
        velocity_y = (translation[1] - prev_translation[1])/dt
        velocity_z = (translation[2] - prev_translation[2])/dt * -1
        print(f"Frame {i}: translation_z={translation[2]}, velocity_z={velocity_z}")

        # Setting the velocities

        odom.twist.twist.linear.x = float(velocity_x)
        odom.twist.twist.linear.y = float(velocity_y)
        odom.twist.twist.linear.z = float(velocity_z)

        angular_velocity = angular_velocity.flatten()  # Ensure it's a 1D array
        odom.twist.twist.angular.x = float(angular_velocity[0])
        odom.twist.twist.angular.y = float(angular_velocity[1])
        odom.twist.twist.angular.z = float(angular_velocity[2])


        return odom

    def odom_truth(self,T,i):

        """
        The same as odom_handle but for the true pose values
        """

        odom, rotation_matrix, translation, quaternion = self.odom_shorten(T)
        print(f"{rotation_matrix.shape}")

        odom, rotation_matrix, translation, quaternion = self.odom_shorten(T)

        odom.pose.pose.position = Point(
            x = translation[0], y = translation[1], z = translation[2])
        odom.pose.pose.orientation = Quaternion(
            x = quaternion[0], y = quaternion[1], z = quaternion[2], w =quaternion[3]
        )   

        current_time = self.get_clock().now()
        self.timestamps_truth.append(current_time)

        if len(self.timestamps_truth) > 1:
            dt = (self.timestamps_truth[-1] - self.timestamps_truth[-2]).nanoseconds * 1e-9

            if dt == 0:  
                dt = 1e-6
        else:
            dt = 0.1  # 
        if len(self.poses) > 1:
            prev_T = self.poses[i-1]  
            prev_translation = prev_T[:3, 3]  
            prev_rotation_matrix = prev_T[:3, :3]  
            rot_diff = rotation_matrix @ prev_rotation_matrix.T
            print(f"rot_diff shape: {rot_diff.shape}")
            U, _, Vt = np.linalg.svd(rot_diff)
            rot_diff = U @ Vt  
            angular_velocity = R.from_matrix(rot_diff).as_rotvec() / dt

        else:

            prev_translation = np.zeros(3)
            prev_rotation_matrix = np.eye(3) 
            angular_velocity = np.zeros(shape= (3,3))

        velocity_x = (translation[0] - prev_translation[0])/dt
        velocity_y = (translation[1] - prev_translation[1])/dt
        velocity_z = (translation[2] - prev_translation[2])/dt  
        print(f"Frame {i}: translation_z={translation[2]}, velocity_z={velocity_z}")

        odom.twist.twist.linear.x = float(velocity_x)
        odom.twist.twist.linear.y = float(velocity_y)
        odom.twist.twist.linear.z = float(velocity_z)

        angular_velocity = angular_velocity.flatten() 
        odom.twist.twist.angular.x = float(angular_velocity[0])
        odom.twist.twist.angular.y = float(angular_velocity[1])
        odom.twist.twist.angular.z = float(angular_velocity[2])

        return odom

    def visualizer(self):

        """
        Visualizing accuracy
        """

        gt_x = [pose[0, 3] for pose in self.poses]
        gt_z = [pose[2, 3] for pose in self.poses]
        est_x = [pose[0, 3] for pose in self.estimates]
        est_z = [pose[2, 3] for pose in self.estimates]

        plt.figure(figsize=(10, 5))
        plt.plot(gt_x, gt_z, label="Ground Truth", color='g')
        plt.plot(est_x, est_z, label="Estimated", color='r', linestyle='dashed')
        plt.legend()
        plt.xlabel("X Position")
        plt.ylabel("Z Position")
        plt.title("Odometry Trajectory Comparison")
        # plt.show()
        min_len = min(len(gt_x), len(est_x))
        gt_x, gt_z = gt_x[:min_len], gt_z[:min_len]
        est_x, est_z = est_x[:min_len], est_z[:min_len]
        import numpy as np

        est_z = np.array(est_z)  # Convert list to NumPy array
        gt_z = np.array(gt_z)

        print("Estimated Z:", est_z[:5])  # First 5 Z-values
        print("Ground Truth Z:", gt_z[:5])  # First 5 Z-values


        errors_x = np.abs(np.array(gt_x) - np.array(est_x))
        errors_z = np.abs(np.array(gt_z) - np.array(est_z))

        print("Mean X Error:", np.mean(errors_x))
        print("Mean Z Error:", np.mean(errors_z))
        print("Max X Error:", np.max(errors_x))
        print("Max Z Error:", np.max(errors_z))

    def odom_shorten(self, T):
        
        """
        Helper function for the odom functions
        """

        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        # Create Odom message

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg() # Get current time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        if rotation_matrix.shape != (3, 3):
            print(f"Unexpected shape: {rotation_matrix.shape}")
        
        return odom, rotation_matrix, translation, quaternion

    def left_handle(self, msg: Image):
        
        """
        Processes received image frames
        """
        i =0
        left = self.ros_to_cv2(msg)
        if self.prev_image is None:
            self.prev_image = left
            return
        pts1, pts2 = self.matching(self.prev_image, left)
            
            # May be needed for future optimization techniques
            
        print("P_l shape:", self.P_l.shape)  
        print("P_r shape:", self.P_r.shape)  

        #pts3D = self.triangulate_points(pts1,pts2)

        # Filtering unnecessary points

        if len(pts1) > 8:
                T_k = self.calc_essential_matrix(pts1, pts2)
                if i > 0:
                    gt_translation = self.poses[i][:3, 3] - self.poses[i-1][:3, 3]
                    scale = np.linalg.norm(gt_translation) / np.linalg.norm(T_k[:3, 3])
                    print(f"Scale factor at frame {i}: {scale}")
                    T_k[:3, 3] = T_k[:3, 3] * scale
                self.C_k = self.C_k @ T_k
                msg = self.odom_handle(self.C_k,i)
                accurates = self.odom_truth(self.poses[i],i)
                print(f"Z error : {accurates.pose.pose.position.z - msg.pose.pose.position.z}")
                # Publishing the messages

                self.pub.publish(msg)
                self.accurate_pub.publish(accurates)
                #time.sleep(1)

                print(f"Pose at frame {i+1}:\n{self.C_k}\n")
                self.estimates.append(self.C_k) 
                i = i+1

    def depth_handle(self, msg: Image):
        depth = msg
def main(args = None):

    """
    Main Entry point of the Visual Odometry node
    """

    rclpy.init(args=args) # Initializing ROS connection
    
    vo = VisualOdometry() # Initializing ROS node
    
    rclpy.spin(vo) # Spinning the node
    
    rclpy.shutdown() # Shutting down ROS connection


if __name__ == "__main__":
    main()