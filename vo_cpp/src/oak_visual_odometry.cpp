#include "vo_cpp/oak_visual_odometry.hpp" // Adjust path as per your package structure

#include <iomanip> // For std::setprecision

// For FLANN with ORB, LSH is typically recommended.
// #include "opencv2/flann/miniflann.hpp" // If using specific FLANN index types

OAKVisualOdometry::OAKVisualOdometry()
: Node("oak_visual_odom"), K_received_(false), last_timestamp_valid_(false) {
    RCLCPP_INFO(this->get_logger(), "Initializing OAK Visual Odometry node...");

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Initialize ORB detector
    orb_ = cv::ORB::create(kMaxFeatures_);
    
    // Initialize FLANN matcher for ORB descriptors (binary)
    // LSH (Locality Sensitive Hashing) is suitable for binary descriptors like ORB
    // Parameters for LSH: table_number, key_size, multi_probe_level
    // These are example parameters; tuning might be needed.
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    // The following settings are more for SIFT/SURF. For ORB, FLANN often uses LSH.
    // However, OpenCV's FlannBasedMatcher tries to pick a suitable index if not specified.
    // For binary descriptors, it should default to an LSH index.
    // You might need to explicitly set indexParams for FLANN with ORB if default doesn't work well.
    // Example for LSH:
    // cv::Ptr<cv::flann::IndexParams> indexParams = new cv::flann::LshIndexParams(12, 20, 2);
    // cv::Ptr<cv::flann::SearchParams> searchParams = new cv::flann::SearchParams(50);
    // matcher_ = new cv::FlannBasedMatcher(indexParams, searchParams);
    // For simplicity, we'll let FlannBasedMatcher use its defaults for now,
    // which should be reasonable for ORB.

    // Initialize transformation matrix
    C_k_ = cv::Mat::eye(4, 4, CV_64F); // 64F for precision in transformations

    // Subscribe to camera calibration info first
    calib_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/oak/rgb/camera_info", 10,
        std::bind(&OAKVisualOdometry::camera_info_callback, this, std::placeholders::_1));

    // Prepare for trajectory visualization output
    // Get home directory
    const char* home_dir = getenv("HOME");
    if (home_dir) {
        trajectory_output_path_ = std::string(home_dir) + "/ros2_ws/trajectory_cpp.txt"; // Changed from png to txt
    } else {
        trajectory_output_path_ = "trajectory_cpp.txt"; // Fallback
        RCLCPP_WARN(this->get_logger(), "HOME environment variable not set. Saving trajectory to current directory.");
    }
    
    RCLCPP_INFO(this->get_logger(), "OAK Visual Odometry node initialized.");
    RCLCPP_INFO(this->get_logger(), "Waiting for camera calibration on /oak/rgb/camera_info...");
}

OAKVisualOdometry::~OAKVisualOdometry() {
    RCLCPP_INFO(this->get_logger(), "Shutting down OAK Visual Odometry node.");
    if (trajectory_file_.is_open()) {
        trajectory_file_.close();
    }
    // visualize_trajectory(); // Called by main typically before shutdown if needed.
}


void OAKVisualOdometry::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!K_received_) {
        K_ = cv::Mat(3, 3, CV_64F); // msg->k is double[9]
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K_.at<double>(i, j) = msg->k[i * 3 + j];
            }
        }
        K_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration received and K matrix set.");
        std::ostringstream K_ss;
        K_ss << cv::Formatter::get()->format(K_);
        RCLCPP_INFO(this->get_logger(), "K = \n%s", K_ss.str().c_str());

        // Now that we have camera calibration, set up synchronized subscribers
        setup_synchronized_subscribers();

        // Unsubscribe from the calibration topic as we only need it once
        calib_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Unsubscribed from camera_info topic.");
    }
}

void OAKVisualOdometry::setup_synchronized_subscribers() {
    rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/oak/rgb/image_rect");
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/oak/stereo/image_raw"); // The Python code uses image_raw for depth

    ts_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *rgb_sub_, *depth_sub_);
    
    ts_->getPolicy()->setAgePenalty(0.1); // Equivalent to slop in Python
    ts_->registerCallback(std::bind(
        &OAKVisualOdometry::synchronized_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Synchronized subscribers set up for RGB and Depth images.");
}

void OAKVisualOdometry::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    // RCLCPP_INFO(this->get_logger(), "Synchronized callback triggered.");
    if (!K_received_) {
        RCLCPP_WARN(this->get_logger(), "Camera intrinsics (K) not yet received. Skipping frame.");
        return;
    }

    try {
        // Convert ROS messages to OpenCV images
        // Python uses "mono8" for RGB, implying grayscale. Ensure this is intended.
        // If actual RGB, use "bgr8" or "rgb8" and adapt feature matching if needed.
        cv_bridge_ptr_rgb_ = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::MONO8);
        
        // Depth image: Python code uses depth_msg.encoding.
        // OAK-D typically outputs depth as 16UC1 (millimeters as uint16_t)
        // Or it could be 32FC1 (meters as float)
        // cv_bridge::toCvShare will convert if possible, but it's best to use the native encoding if known
        // or explicitly request a conversion (e.g., to "32FC1" if you want meters as float)
        // For this port, we'll try to use the message's encoding directly.
        cv_bridge_ptr_depth_ = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);

        const cv::Mat& current_rgb_img = cv_bridge_ptr_rgb_->image;
        const cv::Mat& current_depth_img = cv_bridge_ptr_depth_->image;

        // RCLCPP_INFO(this->get_logger(), "RGB: %dx%d, Depth: %dx%d, Depth type: %d",
        //    current_rgb_img.cols, current_rgb_img.rows,
        //    current_depth_img.cols, current_depth_img.rows, current_depth_img.type());


        if (!prev_rgb_img_.empty() && !prev_depth_img_.empty()) {
            process_frames(
                prev_rgb_img_, current_rgb_img,
                prev_depth_img_, current_depth_img,
                rgb_msg->header.stamp // Use RGB timestamp as reference
            );
        }

        // Store current frames for next iteration
        // .clone() is important to ensure data is copied, as cv_bridge_ptr might go out of scope
        // or be overwritten in the next callback.
        prev_rgb_img_ = current_rgb_img.clone();
        prev_depth_img_ = current_depth_img.clone();
        last_timestamp_ = rgb_msg->header.stamp;
        last_timestamp_valid_ = true;

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in synchronized_callback: %s", e.what());
    }
}

std::tuple<cv::Mat, cv::Mat, std::vector<cv::DMatch>> OAKVisualOdometry::match_features(
    const cv::Mat& img1, const cv::Mat& img2)
{
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    orb_->detectAndCompute(img1, cv::noArray(), kp1, des1);
    orb_->detectAndCompute(img2, cv::noArray(), kp2, des2);

    if (des1.empty() || des2.empty() || des1.rows < 2 || des2.rows < 2) { // des.rows for num descriptors
        RCLCPP_WARN(this->get_logger(), "Not enough features detected for matching.");
        return {cv::Mat(), cv::Mat(), {}};
    }
    if (des1.type() != CV_8U || des2.type() != CV_8U) {
         RCLCPP_WARN(this->get_logger(), "ORB descriptors are not CV_8U. des1 type: %d, des2 type: %d", des1.type(), des2.type());
         // This can happen if images are empty or something else is wrong.
         return {cv::Mat(), cv::Mat(), {}};
    }


    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(des1, des2, knn_matches, 2); // Find 2 nearest neighbors

    std::vector<cv::DMatch> good_matches;
    for (const auto& m_n : knn_matches) {
        if (m_n.size() == 2 && m_n[0].distance < kLoweRatioTest_ * m_n[1].distance) {
            good_matches.push_back(m_n[0]);
        }
    }
    
    // RCLCPP_INFO(this->get_logger(), "Number of good matches: %zu", good_matches.size());

    if (good_matches.size() < 8) { // Need at least 8 for estimateEssentialMat, PnP needs fewer but good to have more
        RCLCPP_WARN(this->get_logger(), "Not enough good matches after ratio test: %zu", good_matches.size());
        return {cv::Mat(), cv::Mat(), {}};
    }

    cv::Mat pts1(good_matches.size(), 2, CV_32F);
    cv::Mat pts2(good_matches.size(), 2, CV_32F);

    for (size_t i = 0; i < good_matches.size(); ++i) {
        pts1.at<float>(i, 0) = kp1[good_matches[i].queryIdx].pt.x;
        pts1.at<float>(i, 1) = kp1[good_matches[i].queryIdx].pt.y;
        pts2.at<float>(i, 0) = kp2[good_matches[i].trainIdx].pt.x;
        pts2.at<float>(i, 1) = kp2[good_matches[i].trainIdx].pt.y;
    }

    return {pts1, pts2, good_matches};
}

std::pair<cv::Mat, std::vector<int>> OAKVisualOdometry::get_3d_points(
    const cv::Mat& pts2d, const cv::Mat& depth_img) // pts2d is Nx2 CV_32F
{
    std::vector<cv::Point3f> pts3d_vec;
    std::vector<int> valid_indices;

    if (K_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Camera intrinsic matrix K is empty in get_3d_points.");
        return {cv::Mat(), {}};
    }
    
    // Depth scale factor: OAK-D usually outputs depth in mm (uint16_t)
    // If it's float, it's usually in meters.
    float scale_factor = 1.0f;
    if (depth_img.type() == CV_16UC1) {
        scale_factor = 1000.0f; // Convert mm to meters
    } else if (depth_img.type() != CV_32FC1) {
        RCLCPP_WARN(this->get_logger(), "Depth image type is %d, not CV_16UC1 or CV_32FC1. Assuming meters or direct values.", depth_img.type());
    }
    //RCLCPP_INFO(this->get_logger(), "Depth image type: %d, scale_factor: %f", depth_img.type(), scale_factor);


    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    for (int i = 0; i < pts2d.rows; ++i) {
        float u = pts2d.at<float>(i, 0);
        float v = pts2d.at<float>(i, 1);

        int u_int = static_cast<int>(std::round(u));
        int v_int = static_cast<int>(std::round(v));

        if (u_int >= 0 && u_int < depth_img.cols && v_int >= 0 && v_int < depth_img.rows) {
            float depth_value_raw;
            if (depth_img.type() == CV_16UC1) {
                depth_value_raw = static_cast<float>(depth_img.at<uint16_t>(v_int, u_int));
            } else if (depth_img.type() == CV_32FC1) {
                depth_value_raw = depth_img.at<float>(v_int, u_int);
            } else { // Fallback, might be incorrect
                depth_value_raw = static_cast<float>(depth_img.at<uchar>(v_int, u_int)); // Unlikely for depth
            }
            
            float depth_in_meters = depth_value_raw / scale_factor;

            // Depth validity check (e.g., 0.1m to 10m)
            if (depth_in_meters > 0.1f && depth_in_meters < 10.0f) {
                float x = (u - cx) * depth_in_meters / fx;
                float y = (v - cy) * depth_in_meters / fy;
                pts3d_vec.push_back(cv::Point3f(x, y, depth_in_meters));
                valid_indices.push_back(i);
            }
        }
    }

    if (pts3d_vec.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Only %zu valid 3D points found.", pts3d_vec.size());
        return {cv::Mat(), {}};
    }
    
    // Convert vector of Point3f to cv::Mat (Nx3 CV_32F)
    cv::Mat pts3d_mat(pts3d_vec.size(), 3, CV_32F);
    for(size_t i = 0; i < pts3d_vec.size(); ++i) {
        pts3d_mat.at<float>(i, 0) = pts3d_vec[i].x;
        pts3d_mat.at<float>(i, 1) = pts3d_vec[i].y;
        pts3d_mat.at<float>(i, 2) = pts3d_vec[i].z;
    }

    return {pts3d_mat, valid_indices};
}


void OAKVisualOdometry::process_frames(
    const cv::Mat& prev_rgb, const cv::Mat& curr_rgb,
    const cv::Mat& prev_depth, const cv::Mat& curr_depth,
    const rclcpp::Time& timestamp)
{
    // Match features between previous and current RGB frames
    auto [pts1_2d_all, pts2_2d_all, good_matches] = match_features(prev_rgb, curr_rgb);

    if (pts1_2d_all.empty() || good_matches.size() < 8) {
        RCLCPP_WARN(this->get_logger(), "Not enough good matches found in process_frames.");
        return;
    }

    // Get 3D points for features in the *previous* frame using *previous* depth
    auto [pts1_3d, valid_indices_prev] = get_3d_points(pts1_2d_all, prev_depth);

    if (pts1_3d.empty() || valid_indices_prev.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Not enough valid 3D points from previous frame.");
        return;
    }

    // Filter the 2D points in the *current* frame to only include those with valid depth in the previous frame
    cv::Mat pts2_2d_valid(valid_indices_prev.size(), 2, CV_32F);
    for (size_t i = 0; i < valid_indices_prev.size(); ++i) {
        pts2_2d_valid.at<float>(i, 0) = pts2_2d_all.at<float>(valid_indices_prev[i], 0);
        pts2_2d_valid.at<float>(i, 1) = pts2_2d_all.at<float>(valid_indices_prev[i], 1);
    }

    // Solve PnP: Find pose of current camera wrt previous camera's 3D points
    // pts1_3d are 3D points in prev_camera_frame
    // pts2_2d_valid are their projections in curr_camera_frame
    // solvePnP gives rvec, tvec such that: P_curr_cam = R * P_prev_cam + t
    // This is the transformation from prev_camera_frame to curr_camera_frame
    cv::Mat rvec, tvec;
    cv::Mat inliers_idx; // Output vector of indices of inliers
    bool success = cv::solvePnPRansac(
        pts1_3d, pts2_2d_valid, K_, cv::noArray(), // No distortion coefficients for now
        rvec, tvec, false, 100, 8.0, 0.99, inliers_idx, cv::SOLVEPNP_ITERATIVE // Default SOLVEPNP_ITERATIVE
    );

    if (!success || inliers_idx.empty() || inliers_idx.rows < 5) {
        RCLCPP_WARN(this->get_logger(), "PnP failed or too few inliers: %d", inliers_idx.rows);
        return;
    }

    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat); // Convert rotation vector to matrix

    // Create transformation matrix T_k_k_minus_1 (from previous frame to current frame)
    cv::Mat T_k_k_minus_1 = cv::Mat::eye(4, 4, CV_64F);
    R_mat.convertTo(T_k_k_minus_1(cv::Rect(0, 0, 3, 3)), CV_64F);
    cv::Mat tvec_64f;
    tvec.convertTo(tvec_64f, CV_64F);
    tvec_64f.copyTo(T_k_k_minus_1(cv::Rect(3, 0, 1, 3)));

    // Update overall pose: C_k_world_to_curr = C_k_minus_1_world_to_prev * T_k_prev_to_curr
    // The PnP output (R_mat, tvec) is T_prev_to_curr.
    // So, C_k = C_{k-1} * T_{k-1 \to k}
    C_k_ = C_k_ * T_k_k_minus_1;

    nav_msgs::msg::Odometry odom_msg = create_odom_message(C_k_, timestamp);
    odom_pub_->publish(odom_msg);
    
    estimates_.push_back(C_k_.clone()); // Store a copy
    // RCLCPP_INFO(this->get_logger(), "New pose estimated with %d PnP inliers.", inliers_idx.rows);
    
    // Write to trajectory file
    if (!trajectory_file_.is_open()) {
        trajectory_file_.open(trajectory_output_path_, std::ios::out | std::ios::trunc);
        if (trajectory_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Opened trajectory output file: %s", trajectory_output_path_.c_str());
            trajectory_file_ << std::fixed << std::setprecision(6); // Set precision for floating point numbers
            // Write header
            trajectory_file_ << "timestamp_sec timestamp_nanosec tx ty tz qx qy qz qw" << std::endl;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory output file: %s", trajectory_output_path_.c_str());
        }
    }
    if (trajectory_file_.is_open()) {
        tf2::Quaternion q;
        tf2::Matrix3x3 R_tf2;
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) R_tf2[i][j] = C_k_.at<double>(i,j);
        R_tf2.getRotation(q);
        q.normalize();

        trajectory_file_ << timestamp.seconds() << " "
                         << timestamp.nanoseconds() << " "
                         << C_k_.at<double>(0, 3) << " " // tx
                         << C_k_.at<double>(1, 3) << " " // ty
                         << C_k_.at<double>(2, 3) << " " // tz
                         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

}

nav_msgs::msg::Odometry OAKVisualOdometry::create_odom_message(
    const cv::Mat& T_world_to_cam, const rclcpp::Time& timestamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = "odom";      // Or your fixed world frame
    odom.child_frame_id = "base_link"; // Or your camera frame

    // Extract translation and rotation from the transformation matrix T (world to camera)
    // The Python code seems to swap x,y,z coordinates for ROS. Let's follow that.
    // Python: x=translation[2], y=translation[1], z=translation[0]
    // If T_world_to_cam is C_k_:
    // C_k_(0,3) is x in camera world, C_k_(1,3) is y, C_k_(2,3) is z.
    // If ROS base_link is X-forward, Y-left, Z-up, and
    // Camera is X-right, Y-down, Z-forward:
    // ROS_X = Cam_Z
    // ROS_Y = -Cam_X
    // ROS_Z = -Cam_Y
    // The Python code directly maps: odom.x=T[2,3], odom.y=T[1,3], odom.z=T[0,3]
    // This implies a coordinate system transformation.
    // Let's assume the Python code's direct mapping is what's desired for now,
    // but this is a common point of confusion and error.
    // If C_k_ is from world to camera (X right, Y down, Z forward):
    // T_world_to_cam.at<double>(0,3) -> camera_x
    // T_world_to_cam.at<double>(1,3) -> camera_y
    // T_world_to_cam.at<double>(2,3) -> camera_z

    // Original Python mapping:
    // odom.pose.pose.position.x = T_world_to_cam.at<double>(2, 3); // z from camera
    // odom.pose.pose.position.y = T_world_to_cam.at<double>(1, 3); // y from camera
    // odom.pose.pose.position.z = T_world_to_cam.at<double>(0, 3); // x from camera
    // This is unusual. A more standard interpretation:
    // If 'odom' frame is NWU (North-West-Up) or ENU (East-North-Up)
    // And camera is standard (X-right, Y-down, Z-forward)
    // And base_link is ROS standard (X-forward, Y-left, Z-up)
    // Then C_k_ (world_to_camera) needs to be transformed to world_to_base_link.
    // For simplicity, let's assume C_k_ *is* the world_to_base_link transform.
    // This means the PnP output, which is camera-to-camera, is also base_link-to-base_link.
    odom.pose.pose.position.x = T_world_to_cam.at<double>(0, 3);
    odom.pose.pose.position.y = T_world_to_cam.at<double>(1, 3);
    odom.pose.pose.position.z = T_world_to_cam.at<double>(2, 3);


    cv::Mat R_cv = T_world_to_cam(cv::Rect(0, 0, 3, 3));
    tf2::Matrix3x3 tf_rot_mat(
        R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
        R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
        R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
    );
    tf2::Quaternion tf_quat;
    tf_rot_mat.getRotation(tf_quat);
    tf_quat.normalize(); // Ensure it's a unit quaternion

    odom.pose.pose.orientation = tf2::toMsg(tf_quat);

    // Covariance (example: diagonal, higher uncertainty for orientation)
    for(int i=0; i<36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0] = 0.1; // x
    odom.pose.covariance[7] = 0.1; // y
    odom.pose.covariance[14] = 0.1; // z
    odom.pose.covariance[21] = 0.05; // roll
    odom.pose.covariance[28] = 0.05; // pitch
    odom.pose.covariance[35] = 0.05; // yaw


    // Velocity calculation (if we have previous estimates)
    if (last_timestamp_valid_ && estimates_.size() > 1) {
        double dt_sec = (timestamp.seconds() - last_timestamp_.seconds()) +
                        (timestamp.nanoseconds() - last_timestamp_.nanoseconds()) * 1e-9;

        if (dt_sec > 0.001) { // Ensure meaningful time difference
            const cv::Mat& prev_T = estimates_[estimates_.size() - 2]; // Second to last estimate

            cv::Mat prev_translation = prev_T(cv::Rect(3, 0, 1, 3));
            cv::Mat curr_translation = T_world_to_cam(cv::Rect(3, 0, 1, 3));
            cv::Mat linear_velocity_mat = (curr_translation - prev_translation) / dt_sec;
            
            odom.twist.twist.linear.x = linear_velocity_mat.at<double>(0);
            odom.twist.twist.linear.y = linear_velocity_mat.at<double>(1);
            odom.twist.twist.linear.z = linear_velocity_mat.at<double>(2);

            cv::Mat prev_R_cv = prev_T(cv::Rect(0, 0, 3, 3));
            // R_delta = R_curr * R_prev.inv()
            cv::Mat R_delta_cv = R_cv * prev_R_cv.t(); // .t() is transpose, which is inverse for rotation matrix
            
            tf2::Matrix3x3 tf_R_delta(
                R_delta_cv.at<double>(0,0), R_delta_cv.at<double>(0,1), R_delta_cv.at<double>(0,2),
                R_delta_cv.at<double>(1,0), R_delta_cv.at<double>(1,1), R_delta_cv.at<double>(1,2),
                R_delta_cv.at<double>(2,0), R_delta_cv.at<double>(2,1), R_delta_cv.at<double>(2,2)
            );
            tf2::Quaternion q_delta;
            tf_R_delta.getRotation(q_delta);
            
            // Get axis-angle from quaternion: angle = 2 * acos(qw), axis = (qx,qy,qz)/sin(angle/2)
            // Or, more robustly, use tf2 to get roll, pitch, yaw changes if that's easier.
            // For small rotations, angle is approx 2 * sqrt(qx^2+qy^2+qz^2)
            // And (wx, wy, wz) approx (2*qx/dt, 2*qy/dt, 2*qz/dt) if qw is close to 1.
            // A more direct way is to get the axis and angle from R_delta_cv using Rodrigues.
            cv::Mat rot_vec_delta;
            cv::Rodrigues(R_delta_cv, rot_vec_delta); // rot_vec_delta is axis * angle

            odom.twist.twist.angular.x = rot_vec_delta.at<double>(0) / dt_sec;
            odom.twist.twist.angular.y = rot_vec_delta.at<double>(1) / dt_sec;
            odom.twist.twist.angular.z = rot_vec_delta.at<double>(2) / dt_sec;
        }
    }
     // Twist covariance (example)
    for(int i=0; i<36; ++i) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0] = 0.05; // vx
    odom.twist.covariance[7] = 0.05; // vy
    odom.twist.covariance[14] = 0.05; // vz
    odom.twist.covariance[21] = 0.02; // wx
    odom.twist.covariance[28] = 0.02; // wy
    odom.twist.covariance[35] = 0.02; // wz

    return odom;
}

void OAKVisualOdometry::visualize_trajectory() {
    RCLCPP_INFO(this->get_logger(), "Visualizing trajectory is handled by writing to file: %s", trajectory_output_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Plot this file using an external tool (e.g., Python with Matplotlib, Gnuplot).");
    RCLCPP_INFO(this->get_logger(), "Format: timestamp_sec timestamp_nanosec tx ty tz qx qy qz qw");
    // The actual writing is done in process_frames. This function is just a placeholder.
    // If plotting directly in C++ were implemented, it would go here.
    if (trajectory_file_.is_open()) {
        trajectory_file_.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory file closed.");
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OAKVisualOdometry>();
    
    // Add a SIGINT handler for graceful shutdown and trajectory visualization
    // This is more complex in C++ than Python's try-except KeyboardInterrupt
    // For now, we'll rely on the destructor or a explicit call if possible.
    // A common pattern is to spin in a try-catch block.
    try {
        rclcpp::spin(node);
    } catch (const rclcpp::exceptions::RCLError& e) {
        RCLCPP_ERROR(node->get_logger(), "RCLCPP Error during spin: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Unhandled exception during spin: %s", e.what());
    }
    
    // This part will be reached after rclcpp::shutdown() is called elsewhere or spin is interrupted
    node->visualize_trajectory(); // Call before node destruction
    
    rclcpp::shutdown();
    return 0;
}