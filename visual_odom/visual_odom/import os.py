def process_sequence(self):

        """
        Processes subsequent image frames
        """

        for i in range(len(self.imgs_l) -1):

            img1, img2 = self.imgs_l[i], self.imgs_l[i + 1]
            pts1, pts2 = self.matching(img1, img2)
            
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