#ifndef OAK_VISUAL_ODOMETRY_HPP_
#define OAK_VISUAL_ODOMETRY_HPP_

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::fromMsg and tf2::toMsg

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

class OAKVisualOdometry : public rclcpp::Node {
public:
    OAKVisualOdometry();
    ~OAKVisualOdometry();
    void setup_synchronized_subscribers(); // Assuming this might be called after construction too
    void visualize_trajectory(); // <----- MOVED HERE (or ensure it is here)

private:
    // Callbacks
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void synchronized_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    // Helper methods
    void process_frames(
        const cv::Mat& prev_rgb, const cv::Mat& curr_rgb,
        const cv::Mat& prev_depth, const cv::Mat& curr_depth,
        const rclcpp::Time& timestamp);
    
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::DMatch>> match_features(
        const cv::Mat& img1, const cv::Mat& img2);

    std::pair<cv::Mat, std::vector<int>> get_3d_points(
        const cv::Mat& pts2d, const cv::Mat& depth_img);
    
    nav_msgs::msg::Odometry create_odom_message(
        const cv::Mat& T, const rclcpp::Time& timestamp);
    

    // ROS Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calib_sub_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> ts_;

    // CV Bridge
    cv_bridge::CvImageConstPtr cv_bridge_ptr_rgb_;
    cv_bridge::CvImageConstPtr cv_bridge_ptr_depth_;

    // ORB and FLANN
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::DescriptorMatcher> matcher_; // Using DescriptorMatcher base class

    // State variables
    cv::Mat K_;                // Camera intrinsic matrix
    bool K_received_ = false;
    cv::Mat C_k_;              // Current transformation matrix (world to camera)
    rclcpp::Time last_timestamp_;
    bool last_timestamp_valid_ = false;
    cv::Mat prev_rgb_img_;
    cv::Mat prev_depth_img_;
    std::vector<cv::Mat> estimates_; // Stores trajectory poses (C_k values)

    // Parameters for ORB and FLANN (can be made ROS parameters)
    const int kMaxFeatures_ = 3000;
    const float kLoweRatioTest_ = 0.8f;
    // FLANN parameters (adjust as needed for CV_8U descriptors from ORB)
    // For ORB descriptors (binary), LSH is often used.
    // If using FlannBasedMatcher with default (KDTreeIndexParams for SIFT/SURF):
    // const int kFlannIndexTrees_ = 5; 
    // const int kFlannSearchChecks_ = 50;
    // For ORB (binary descriptors), LSH is better:
    // cv::flann::LshIndexParams index_params = cv::flann::LshIndexParams(12, 20, 2);


    // For trajectory visualization output
    std::string trajectory_output_path_;
    std::ofstream trajectory_file_;
};

#endif // OAK_VISUAL_ODOMETRY_HPP_