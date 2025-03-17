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