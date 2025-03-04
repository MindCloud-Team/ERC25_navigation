#################                        STEREO VISUAL ODOMETRY CODE                        ###############

#                                       Note: ROS isnt implemented yet.

#                                   Testing was done using the KITTI dataset.


import cv2
import os
import matplotlib.pyplot as plt
import scipy.optimize as opt
import numpy as np
class VisualOdometry:


    def __init__(self):
        
        # Extracting images from the training folders

        self.imgs_l = self.load_images("archive/training/image_2")
        self.imgs_r = self.load_images("archive/training/image_3")

        # Initializing ORB and FLANN parameters
        
        self.orb = cv2.ORB.create(2000)
        self.index_params = dict(algorithm=6, trees=5)
        self.search_params = dict(checks=50)
        
        filepath = "/home/ibrahim/ros2_ws/src/Visual_Odometry/KITTI_sequence_1/calib.txt"
        self.K_l, self.P_l, self.K_r, self.P_r = self.load_calibration(filepath)
        
        # Generating the depth map

        # May be needed if we're dividing the image if we use the FAST detector

        block = 11
        P1 = block * block * 8
        P2 = block * block * 32
        self.disparity = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=32, blockSize=block, P1=P1, P2=P2
        )

        self.C_k = np.eye(4) 
        self.estimates = []

    def load_images(self, filepath):
        
        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]
        images = [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]
        return images

    def matching(self, img1, img2):
        # Calculating Keypoints of the features in both frames
        
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)

        # Matching using FLANN

        flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Storing the good matches

        good_matches = [m for m, n in matches if m.distance < 0.8 * n.distance]
        matched_image = cv2.drawMatches(img1,
                                        kp1,
                                        img2, 
                                        kp2, 
                                        good_matches, 
                                        None)
        cv2.imshow("Test",matched_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows

        # Extracting the matrix of points for later use

        pts1, pts2 = self.extract_matched_points(kp1, kp2, good_matches)
        return pts1, pts2

    def extract_matched_points(self, kp1, kp2, matches):
        
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        return pts1, pts2

    def calc_essential_matrix(self, pts1, pts2):
        
        # Calculating the Essential Matrix

        E, mask = cv2.findEssentialMat(pts1, pts2, self.K_l, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        # Decomposing the Essential Matrix

        _, R, T, mask = cv2.recoverPose(E, pts1, pts2, self.K_l)

        T_mat = np.eye(4)
        T_mat[:3, :3] = R
        T_mat[:3, 3] = T.flatten()
        return T_mat

    def triangulate_points(self, pts1, pts2):
        
        # Generating the 3D points

        pts1_norm = cv2.undistortPoints(pts1, self.K_l, None)
        pts2_norm = cv2.undistortPoints(pts2, self.K_r, None)

        points_4d = cv2.triangulatePoints(self.P_l, self.P_r, pts1_norm.T, pts2_norm.T)
        points_3d = points_4d[:3, :] / points_4d[3, :]
        return points_3d.T

    def load_calibration(self, filepath):
        
        # Extracting calibration data of the KITTI dataset

        with open(filepath, 'r') as f:
            lines = f.readlines()

        P_l = np.reshape(np.fromstring(lines[0], dtype=np.float64, sep=' '), (3, 4))
        P_r = np.reshape(np.fromstring(lines[1], dtype=np.float64, sep=' '), (3, 4))
        K_l = P_l[:, :3]
        K_r = P_r[:, :3]
        return K_l, P_l, K_r, P_r
    
    def process_sequence(self):

        # Processing each frame incrementally 

        for i in range(len(self.imgs_l) - 1):

            img1, img2 = self.imgs_l[i], self.imgs_l[i + 1]
            pts1, pts2 = self.matching(img1, img2)
            
            # May be needed for future optimization techniques
            
            pts3D = self.triangulate_points(pts1,pts2)

            # Filtering unnecessary points
            if len(pts1) > 8:
                T_k = self.calc_essential_matrix(pts1, pts2)
                self.C_k = self.C_k @ T_k
                print(f"Pose at frame {i+1}:\n{self.C_k}\n")
                self.estimates.append(self.C_k)

def main():
    vo = VisualOdometry()
    vo.process_sequence()

if __name__ == "__main__":
    main()
