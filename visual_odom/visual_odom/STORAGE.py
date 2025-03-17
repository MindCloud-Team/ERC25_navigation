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
        
        # Extracting images from the training folders

        self.imgs_l = self.load_images("/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/image_2")
        self.imgs_r = self.load_images("/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/image_3")

        # Initializing ORB and FLANN parameters
        
        self.orb = cv2.ORB.create(4000)
        self.index_params = dict(algorithm=6, trees=5)
        self.search_params = dict(checks=50)
        
        filepath = "/home/ibrahim/ws/src/visual_odom/kitti_sequence_00/calib.txt"
        self.K_l, self.P_l, self.K_r, self.P_r, self.T_r = self.load_calibration(filepath)
        
        # Generating the depth map

        # May be needed if we're dividing the image if we use the FAST detector

        block = 11
        P1 = block * block * 8
        P2 = block * block * 32
        self.disparity = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=96, blockSize=block, P1=P1, P2=P2
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

    def estimate_motion_PnP(self, prev_3D_points, curr_2D_points, camera_matrix):
        """
        Estimate motion using PnP RANSAC with improved error handling and fallback options.
        """
        if len(prev_3D_points) < 8:
            print("Not enough points for PnP estimation")
            return np.eye(4)  # Return identity if not enough points
        
        # Filter out any extreme values in 3D points
        valid_indices = []
        for i, point in enumerate(prev_3D_points):
            # Check if point coordinates are within reasonable bounds
            if np.all(np.abs(point) < 1000):
                valid_indices.append(i)
        
        if len(valid_indices) < 8:
            print(f"Warning: Too few valid 3D points after filtering: {len(valid_indices)}")
            return np.eye(4)  # Return identity if not enough points
        
        # Keep only the valid points
        prev_3D_points = prev_3D_points[valid_indices]
        curr_2D_points = curr_2D_points[valid_indices]
        print(f"Number of 3D Points: {len(prev_3D_points)}")
        print(f"Number of 2D Points: {len(curr_2D_points)}")

        prev_3D_points = np.array(prev_3D_points, dtype=np.float32)
        curr_2D_points = np.array(curr_2D_points, dtype=np.float32)
        
        try:
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                prev_3D_points, curr_2D_points, camera_matrix, distCoeffs=None, 
                flags=cv2.SOLVEPNP_ITERATIVE, 
                reprojectionError=0.5,  # More permissive threshold
                confidence=0.99,
                iterationsCount=100  # More iterations for better results
            )
            if not success:
                print("PnP failed to find a valid pose.")
            print(f"Length: {len(inliers)}")
            if not success or inliers is None or len(inliers) < 6:
                print("PnP estimation failed or had too few inliers")
                return np.eye(4)  # Return identity if PnP fails
            
            # Filter only inlier points
            inliers = inliers.ravel()
            prev_3D_inliers = prev_3D_points[inliers]
            curr_2D_inliers = curr_2D_points[inliers]
            
            try:
                # Refine the pose using only inliers
                rvec, tvec = cv2.solvePnPRefineLM(
                    prev_3D_inliers, curr_2D_inliers, camera_matrix, None, rvec, tvec
                )
            except cv2.error:
                print("PnP refinement failed, using initial estimate")
                # Continue with the unrefined estimate
            
            # Convert rvec to rotation matrix
            R, _ = cv2.Rodrigues(rvec)
            
            # Check if R is a valid rotation matrix
            if np.linalg.det(R) < 0 or abs(np.linalg.det(R) - 1.0) > 0.1:
                print("Invalid rotation matrix detected, correcting...")
                # Correct the rotation matrix using SVD
                U, _, Vt = np.linalg.svd(R)
                R = U @ Vt
            
            # Scale control: limit extreme translations
            max_translation = 10.0  # Maximum allowed translation magnitude
            t_mag = np.linalg.norm(tvec)
            if t_mag > max_translation:
                tvec = tvec * (max_translation / t_mag)
                print(f"Translation scaled down from {t_mag} to {max_translation}")
            
            # Form the transformation matrix
            T_mat = np.eye(4)
            T_mat[:3, :3] = R
            T_mat[:3, 3] = tvec.flatten()
            
            return T_mat
            
        except cv2.error as e:
            print(f"OpenCV error in PnP estimation: {str(e)}")
            return np.eye(4)  # Return identity if error occurs
        except Exception as e:
            print(f"Unexpected error in PnP estimation: {str(e)}")
            return np.eye(4)  # Return identity if any error occur


    def triangulate_points(self, pts1, pts2):
        """
        Triangulates 3D points from stereo image correspondences.
        :param pts1: Nx2 array of points in the left image.
        :param pts2: Nx2 array of points in the right image.
        :return: Nx3 array of triangulated 3D points.
        """
        # Convert to homogeneous coordinates (2xN format for OpenCV)
        pts1 = np.asarray(pts1, dtype=np.float32).T
        pts2 = np.asarray(pts2, dtype=np.float32).T
        
        # Check input dimensions
        if pts1.shape[0] != 2 or pts2.shape[0] != 2:
            raise ValueError("Input point arrays must have shape (N,2).")
        print(f"Shape of pts1_l: {pts1.shape}, Shape of pts1_r: {pts2.shape}")
        print(f"P_l: {self.P_l}\nP_r: {self.P_r}")

        pts_4d_hom = cv2.triangulatePoints(self.P_l, self.P_r, pts1.T, pts2.T)
        print(f"Raw triangulated points (homogeneous): {pts_4d_hom}")

        pts_3d = pts_4d_hom[:3] / pts_4d_hom[3]
        print(f"Triangulated points (3D): {pts_3d}")

        
        # Convert from homogeneous coordinates to 3D
        points_3d = pts_4d_hom[:3, :] / (pts_4d_hom[3, :] + 1e-10)  # Avoid division by zero
        
        # Transpose to Nx3 format
        points_3d = points_3d.T
        
        # Debugging: Print some statistics
        print("First 5 triangulated points (homogeneous):", pts_4d_hom[:, :5])
        print("First 5 triangulated points (3D):", points_3d[:5])
        print("Min 3D Point:", np.min(points_3d, axis=0))
        print("Max 3D Point:", np.max(points_3d, axis=0))
        
        return points_3d

    def load_calibration(self,filepath):
        """
        Extracts KITTI calibration data from a file.
        Returns: 
            K_l (3x3): Left camera intrinsic matrix
            P_l (3x4): Left camera projection matrix
            K_r (3x3): Right camera intrinsic matrix
            P_r (3x4): Right camera projection matrix
            Tr (3x4, optional): Transformation matrix (LiDAR to camera)
        """
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # Dictionary to store parsed matrices
        calib_data = {}

        for line in lines:
            parts = line.strip().split(":")
            if len(parts) < 2:
                continue  # Skip malformed lines
            
            key = parts[0]
            values = np.fromstring(parts[1], dtype=np.float64, sep=' ')
            
            if key in ["P0", "P1", "P2", "P3"]:
                calib_data[key] = values.reshape(3, 4)
            elif key == "Tr":  # Transformation matrix
                calib_data[key] = values.reshape(3, 4)  # Use 3x4, ignore the last row

        # Extract required matrices
        P_l = calib_data.get("P2", None)  # Left camera projection
        P_r = calib_data.get("P3", None)  # Right camera projection
        K_l = P_l[:, :3] if P_l is not None else None  # Left intrinsic
        K_r = P_r[:, :3] if P_r is not None else None  # Right intrinsic
        Tr = calib_data.get("Tr", None)  # Transformation matrix

        return K_l, P_l, K_r, P_r, Tr

    
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
    
    def estimate_motion_PnP(self, prev_3D_points, curr_2D_points, camera_matrix):
        """
        Estimate motion using PnP RANSAC with improved error handling and fallback options.
        """
        if len(prev_3D_points) < 8:
            print("Not enough points for PnP estimation")
            return np.eye(4)  # Return identity if not enough points
        
        # Filter out any extreme values in 3D points - with KITTI-appropriate thresholds
        valid_indices = []
        for i, point in enumerate(prev_3D_points):
            # Check if point coordinates are within reasonable bounds for KITTI
            if (0 < point[2] < 80 and 
                np.abs(point[0]) < 50 and 
                np.abs(point[1]) < 10):
                valid_indices.append(i)
        
        if len(valid_indices) < 8:
            print(f"Warning: Too few valid 3D points after filtering: {len(valid_indices)}")
            return np.eye(4)  # Return identity if not enough points
        
        # Keep only the valid points
        prev_3D_points = prev_3D_points[valid_indices]
        curr_2D_points = curr_2D_points[valid_indices]
        print(f"Number of 3D Points: {len(prev_3D_points)}")
        print(f"Number of 2D Points: {len(curr_2D_points)}")

        prev_3D_points = np.array(prev_3D_points, dtype=np.float32)
        curr_2D_points = np.array(curr_2D_points, dtype=np.float32)
        
        try:
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                prev_3D_points, curr_2D_points, camera_matrix, distCoeffs=None, 
                flags=cv2.SOLVEPNP_ITERATIVE, 
                reprojectionError=2.0,  # Increased threshold for KITTI scale
                confidence=0.99,
                iterationsCount=200  # More iterations for better results
            )
            if not success:
                print("PnP failed to find a valid pose.")
                return np.eye(4)
                
            print(f"Length: {len(inliers) if inliers is not None else 0}")
            if not success or inliers is None or len(inliers) < 6:
                print("PnP estimation failed or had too few inliers")
                return np.eye(4)  # Return identity if PnP fails
            
            # Filter only inlier points
            inliers = inliers.ravel()
            prev_3D_inliers = prev_3D_points[inliers]
            curr_2D_inliers = curr_2D_points[inliers]
            
            try:
                # Refine the pose using only inliers
                rvec, tvec = cv2.solvePnPRefineLM(
                    prev_3D_inliers, curr_2D_inliers, camera_matrix, None, rvec, tvec
                )
            except cv2.error:
                print("PnP refinement failed, using initial estimate")
                # Continue with the unrefined estimate
            
            # Convert rvec to rotation matrix
            R, _ = cv2.Rodrigues(rvec)
            
            # Check if R is a valid rotation matrix
            if np.linalg.det(R) < 0 or abs(np.linalg.det(R) - 1.0) > 0.1:
                print("Invalid rotation matrix detected, correcting...")
                # Correct the rotation matrix using SVD
                U, _, Vt = np.linalg.svd(R)
                R = U @ Vt
            
            # Scale control: limit extreme translations based on KITTI scale
            max_translation = 5.0  # Maximum allowed translation magnitude for KITTI per frame
            t_mag = np.linalg.norm(tvec)
            if t_mag > max_translation:
                tvec = tvec * (max_translation / t_mag)
                print(f"Translation scaled down from {t_mag} to {max_translation}")
            
            # Form the transformation matrix
            T_mat = np.eye(4)
            T_mat[:3, :3] = R
            T_mat[:3, 3] = tvec.flatten()
            
            return T_mat
            
        except cv2.error as e:
            print(f"OpenCV error in PnP estimation: {str(e)}")
            return np.eye(4)  # Return identity if error occurs
        except Exception as e:
            print(f"Unexpected error in PnP estimation: {str(e)}")
            return np.eye(4)  # Return identity if any error occur
    def process_sequence(self):
        """
        Processes subsequent image frames with improved robustness and error handling
        """
        
        for i in range(len(self.imgs_l) - 1):
            try:
                img1_l, img1_r = self.imgs_l[i], self.imgs_r[i]
                
                # Match features between left and right images of the same timestamp
                pts1_l, pts1_r = self.matching(img1_l, img1_r)
                print("Matched")
                # Triangulate 3D points from stereo pair
                pts3D = self.triangulate_points(pts1_l, pts1_r)
                print("Triangulated")
                if len(pts3D) < 8:
                    print(f"Frame {i}: Not enough valid 3D points, skipping")
                    # If not enough valid points, use previous pose estimate
                    if len(self.estimates) > 0:
                        self.C_k = self.estimates[-1].copy()
                    else:
                        self.C_k = np.eye(4)
                    self.estimates.append(self.C_k.copy())
                    continue
                    
                # If this is not the first frame, match with previous frame for motion estimation
                if i > 0:
                    img0_l = self.imgs_l[i-1]
                    pts0_l, pts1_l_temp = self.matching(img0_l, img1_l)
                    
                    # Estimate motion using PnP
                    T_k = self.estimate_motion_PnP(pts3D, pts1_l_temp, self.K_l)
                    
                    # Check if transformation is valid
                    if T_k is None or np.isnan(T_k).any() or np.isinf(T_k).any():
                        print(f"Frame {i}: Invalid transformation matrix, using identity")
                        T_k = np.eye(4)
                    
                    # Scale estimation (using ground truth for now, but could be replaced)
                    if not np.array_equal(T_k, np.eye(4)):
                        gt_translation = self.poses[i][:3, 3] - self.poses[i-1][:3, 3]
                        est_translation_mag = np.linalg.norm(T_k[:3, 3])
                        
                        if est_translation_mag > 1e-6:  # Avoid division by very small numbers
                            scale = np.linalg.norm(gt_translation) / est_translation_mag
                            # Limit scale to reasonable values
                            scale = np.clip(scale, 0.1, 10.0)
                            T_k[:3, 3] = T_k[:3, 3] * scale
                            print(f"Scale factor at frame {i}: {scale}")
                        else:
                            print(f"Frame {i}: Translation magnitude too small, keeping unscaled")
                    
                    # Update cumulative transformation
                    self.C_k = self.C_k @ T_k
                    
                    # Create and publish odometry messages
                    msg = self.odom_handle(self.C_k, i)
                    accurates = self.odom_truth(self.poses[i], i)
                    
                    # Store for visualization
                    self.estimates.append(self.C_k.copy())
                    
                    # Print info
                    print(f"Pose at frame {i+1}:\n{self.C_k}\n")
                    
                    # Publish
                    self.pub.publish(msg)
                    self.accurate_pub.publish(accurates)
                
                else:
                    # For the first frame, just store initial 3D points
                    print(f"Frame {i}: Initializing")
                    self.estimates.append(self.C_k.copy())
                    
            except Exception as e:
                print(f"Error processing frame {i}: {str(e)}")
                # If error occurs, use previous pose estimate
                if len(self.estimates) > 0:
                    self.C_k = self.estimates[-1].copy()
                else:
                    self.C_k = np.eye(4)
                self.estimates.append(self.C_k.copy())


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


def main(args = None):

    """
    Main Entry point of the Visual Odometry node
    """

    rclpy.init(args=args) # Iinitializing ROS connection
    
    vo = VisualOdometry() # Initializing ROS node
    
    rclpy.spin(vo) # Spinning the node
    
    rclpy.shutdown() # Shutting down ROS connection


if __name__ == "__main__":
    main()