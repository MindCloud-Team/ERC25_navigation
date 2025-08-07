#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <rovers_interfaces/msg/aruco_markers.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
#include <map>
#include <vector>
#include <chrono>

class ArucoTagDetector : public rclcpp::Node {
public:
    ArucoTagDetector() : Node("aruco_tag_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        // Parameters
        this->declare_parameter("marker_size", 0.05);
        this->marker_size_ = this->get_parameter("marker_size").as_double();

        this->declare_parameter("aruco_dictionary", "DICT_4X4_100");
        this->aruco_dict_name_ = this->get_parameter("aruco_dictionary").as_string();

        this->declare_parameter("lidar_topic", "/lidar/velodyne_points");
        this->declare_parameter("camera_topics", std::vector<std::string>{
            "/front_cam/zed_node/rgb/image_rect_color", 
            "/back_cam/zed_node/rgb/image_rect_color", 
            "/left_cam/zed_node/rgb/image_rect_color", 
            "/right_cam/zed_node/rgb/image_rect_color"
        });

        this->get_parameter("camera_topics", camera_topics_);
        this->lidar_topic_ = this->get_parameter("lidar_topic").as_string();

        // Subscriptions
        for (const auto &topic : camera_topics_) {
            image_subscribers_.emplace_back(this->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                [this, topic](sensor_msgs::msg::Image::ConstSharedPtr msg){
                    this->image_callback(msg, topic);
                }
            ));
        }

        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ArucoTagDetector::lidar_callback, this, std::placeholders::_1)
        );

        // Publishers
        poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
        aruco_markers_publisher_ = this->create_publisher<rovers_interfaces::msg::ArucoMarkers>("aruco_markers", 10);
        marker_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aruco_markers_viz", 10);

        // Initialize ArUco dictionary
        aruco_dictionary_ = get_aruco_dictionary(aruco_dict_name_);
        
        // Log initialization info
        RCLCPP_INFO(this->get_logger(), "ArUco Tag Detector initialized successfully!");
        RCLCPP_INFO(this->get_logger(), "ArUco dictionary: %s", aruco_dict_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Marker size: %.3f meters", marker_size_);
        RCLCPP_INFO(this->get_logger(), "LiDAR topic: %s", lidar_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera topics:");
        for (const auto &topic : camera_topics_) {
            RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for camera and LiDAR data...");
        
        // Add a timer to show periodic status (every 10 seconds)
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&ArucoTagDetector::print_status, this)
        );
        
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &camera_topic) {
        try {
            // Update image counter
            image_count_++;
            
            RCLCPP_DEBUG(this->get_logger(), "Received image from %s (size: %dx%d)", 
                        camera_topic.c_str(), msg->width, msg->height);
            
            // Convert ROS image to OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Detect markers
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, aruco_dictionary_, corners, marker_ids);

            if (!marker_ids.empty()) {
                // Log detections less frequently to improve performance
                static uint64_t detection_log_counter = 0;
                if (++detection_log_counter % 10 == 0) {
                }
                
                // Get camera frame from topic name
                std::string camera_frame = get_camera_frame(camera_topic);
                
                // Get default camera calibration (you should load these from calibration files)
                cv::Mat camera_matrix = get_camera_matrix();
                cv::Mat dist_coeffs = get_distortion_coeffs();
                
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix, dist_coeffs, rvecs, tvecs);

                // Process each detected marker
                for (size_t i = 0; i < marker_ids.size(); ++i) {
                    // Calculate detection confidence based on marker size and corner geometry
                    double confidence = calculate_detection_confidence(corners[i], image.size());
                    
                    // Create marker detection with camera info
                    MarkerDetection detection;
                    detection.marker_id = marker_ids[i];
                    detection.camera_topic = camera_topic;
                    detection.camera_frame = camera_frame;
                    detection.timestamp = msg->header.stamp;
                    detection.corners = corners[i];
                    detection.rvec = rvecs[i];
                    detection.tvec = tvecs[i];
                    detection.last_seen = this->get_clock()->now();
                    detection.confidence = confidence;
                    
                    // Store/update detection for fusion
                    std::lock_guard<std::mutex> lock(detections_mutex_);
                    recent_detections_[marker_ids[i]] = detection;
                }
                
                // Publish fused results
                publish_fused_poses();
            } else {
                // Count images without markers for status reporting
                images_without_markers_++;
                RCLCPP_DEBUG(this->get_logger(), "No ArUco markers detected in image from %s", camera_topic.c_str());
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        try {
            // Update LiDAR counter
            lidar_count_++;
            
            // Convert to PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
            
            // Store latest LiDAR data
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            latest_lidar_cloud_ = cloud;
            latest_lidar_timestamp_ = msg->header.stamp;
            
            RCLCPP_DEBUG(this->get_logger(), "Received LiDAR cloud with %zu points", cloud->size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "LiDAR callback exception: %s", e.what());
        }
    }


    struct MarkerDetection {
        int marker_id;
        std::string camera_topic;
        std::string camera_frame;
        rclcpp::Time timestamp;
        std::vector<cv::Point2f> corners;
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        rclcpp::Time last_seen;
        geometry_msgs::msg::Pose filtered_pose;
        bool pose_initialized = false;
        double confidence = 0.0;
    };
    
    struct PoseFilter {
        geometry_msgs::msg::Pose current_pose;
        std::vector<geometry_msgs::msg::Pose> pose_history;
        static const size_t HISTORY_SIZE = 5;
        double alpha = 0.3; // Low-pass filter coefficient
        bool initialized = false;
        
        geometry_msgs::msg::Pose filter_pose(const geometry_msgs::msg::Pose& new_pose, double confidence) {
            if (!initialized) {
                current_pose = new_pose;
                initialized = true;
                pose_history.push_back(new_pose);
                return current_pose;
            }
            
            // Calculate distance from current pose to detect outliers
            double distance = sqrt(pow(new_pose.position.x - current_pose.position.x, 2) +
                                 pow(new_pose.position.y - current_pose.position.y, 2) +
                                 pow(new_pose.position.z - current_pose.position.z, 2));
            
            // Reject outliers (sudden jumps > 1 meter)
            if (distance > 1.0 && confidence < 0.8) {
                return current_pose; // Return previous pose for outliers
            }
            
            // Apply exponential moving average with confidence weighting
            double weighted_alpha = alpha * confidence;
            current_pose.position.x = (1.0 - weighted_alpha) * current_pose.position.x + weighted_alpha * new_pose.position.x;
            current_pose.position.y = (1.0 - weighted_alpha) * current_pose.position.y + weighted_alpha * new_pose.position.y;
            current_pose.position.z = (1.0 - weighted_alpha) * current_pose.position.z + weighted_alpha * new_pose.position.z;
            
            // SLERP for orientation smoothing
            tf2::Quaternion q_current, q_new;
            tf2::fromMsg(current_pose.orientation, q_current);
            tf2::fromMsg(new_pose.orientation, q_new);
            tf2::Quaternion q_filtered = q_current.slerp(q_new, weighted_alpha);
            current_pose.orientation = tf2::toMsg(q_filtered);
            
            // Update history
            pose_history.push_back(new_pose);
            if (pose_history.size() > HISTORY_SIZE) {
                pose_history.erase(pose_history.begin());
            }
            
            return current_pose;
        }
    };

    std::string get_camera_frame(const std::string &camera_topic) {
        // Extract camera frame from topic name based on actual TF tree structure
        // Using center_optical_frame which is what the camera_info uses
        if (camera_topic.find("front_cam") != std::string::npos) return "front_cam_center_optical_frame";
        if (camera_topic.find("back_cam") != std::string::npos) return "back_cam_center_optical_frame";
        if (camera_topic.find("left_cam") != std::string::npos) return "left_cam_center_optical_frame";
        if (camera_topic.find("right_cam") != std::string::npos) return "right_cam_center_optical_frame";
        return "front_cam_center_optical_frame"; // Default fallback
    }

    cv::Mat get_camera_matrix() {
        // Real ZED camera calibration data from camera_info
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
            672.1992301940918, 0.0, 960.0,     // fx, 0, cx
            0.0, 672.1992588043213, 540.0,     // 0, fy, cy  
            0.0, 0.0, 1.0);                    // 0, 0, 1
        return camera_matrix;
    }

    cv::Mat get_distortion_coeffs() {
        // Default distortion coefficients - replace with actual calibration data
        return cv::Mat::zeros(4, 1, CV_64F);
    }

    cv::Ptr<cv::aruco::Dictionary> get_aruco_dictionary(const std::string &dict_name) {
        // Map string names to OpenCV ArUco dictionary types
        if (dict_name == "DICT_4X4_50") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        } else if (dict_name == "DICT_4X4_100") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        } else if (dict_name == "DICT_4X4_250") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        } else if (dict_name == "DICT_4X4_1000") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        } else if (dict_name == "DICT_5X5_50") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
        } else if (dict_name == "DICT_5X5_100") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
        } else if (dict_name == "DICT_5X5_250") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
        } else if (dict_name == "DICT_5X5_1000") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
        } else if (dict_name == "DICT_6X6_50") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
        } else if (dict_name == "DICT_6X6_100") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        } else if (dict_name == "DICT_6X6_250") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        } else if (dict_name == "DICT_6X6_1000") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        } else if (dict_name == "DICT_7X7_50") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
        } else if (dict_name == "DICT_7X7_100") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
        } else if (dict_name == "DICT_7X7_250") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
        } else if (dict_name == "DICT_7X7_1000") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
        } else if (dict_name == "DICT_ARUCO_ORIGINAL") {
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown ArUco dictionary '%s', using default DICT_4X4_50", dict_name.c_str());
            return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }
    }

    double calculate_detection_confidence(const std::vector<cv::Point2f>& corners, const cv::Size& image_size) {
        // Calculate the perimeter of the marker
        double perimeter = cv::arcLength(corners, true);
        
        // Confidence can be based on perimeter size relative to image size
        double max_dimension = std::max(image_size.width, image_size.height);
        double confidence = std::clamp(perimeter / max_dimension, 0.0, 1.0);
        
        return confidence;
    }

    geometry_msgs::msg::Pose process_marker_detection(const MarkerDetection &detection) {
        geometry_msgs::msg::Pose pose;
        
        try {
            // Create pose in camera frame
            geometry_msgs::msg::PoseStamped camera_pose;
            camera_pose.header.stamp = detection.timestamp;
            camera_pose.header.frame_id = detection.camera_frame;
            
            camera_pose.pose.position.x = detection.tvec[0];
            camera_pose.pose.position.y = detection.tvec[1];
            camera_pose.pose.position.z = detection.tvec[2];
            
            // Convert rotation vector to quaternion
            cv::Mat rotation_matrix;
            cv::Rodrigues(detection.rvec, rotation_matrix);
            tf2::Quaternion q;
            tf2::Matrix3x3 m(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
            );
            m.getRotation(q);
            
            camera_pose.pose.orientation.x = q.x();
            camera_pose.pose.orientation.y = q.y();
            camera_pose.pose.orientation.z = q.z();
            camera_pose.pose.orientation.w = q.w();
            
            // Transform to odom frame (better coordinate alignment)
            geometry_msgs::msg::PoseStamped odom_pose;
            try {
                tf_buffer_.transform(camera_pose, odom_pose, "odom", tf2::durationFromSec(0.1));
                
                // Correct distance using LiDAR
                double corrected_distance = get_lidar_distance(odom_pose.pose.position);
                if (corrected_distance > 0) {
                    // Scale the position vector to match LiDAR distance
                    double original_distance = sqrt(pow(odom_pose.pose.position.x, 2) + 
                                                   pow(odom_pose.pose.position.y, 2) + 
                                                   pow(odom_pose.pose.position.z, 2));
                    double scale_factor = corrected_distance / original_distance;
                    
                    odom_pose.pose.position.x *= scale_factor;
                    odom_pose.pose.position.y *= scale_factor;
                    odom_pose.pose.position.z *= scale_factor;
                    
                    RCLCPP_DEBUG(this->get_logger(), "Corrected marker %d distance from %.2f to %.2f meters", 
                               detection.marker_id, original_distance, corrected_distance);
                }
                
                // Apply pose filtering to reduce jitter
                {
                    std::lock_guard<std::mutex> filter_lock(filters_mutex_);
                    if (pose_filters_.find(detection.marker_id) == pose_filters_.end()) {
                        pose_filters_[detection.marker_id] = PoseFilter();
                    }
                    pose = pose_filters_[detection.marker_id].filter_pose(odom_pose.pose, detection.confidence);
                }
                
            } catch (const tf2::TransformException &ex) {
                // Apply filtering even for camera frame fallback
                {
                    std::lock_guard<std::mutex> filter_lock(filters_mutex_);
                    if (pose_filters_.find(detection.marker_id) == pose_filters_.end()) {
                        pose_filters_[detection.marker_id] = PoseFilter();
                    }
                    pose = pose_filters_[detection.marker_id].filter_pose(camera_pose.pose, detection.confidence);
                }
            }
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing marker detection: %s", e.what());
        }
        
        return pose;
    }

    double get_lidar_distance(const geometry_msgs::msg::Point &target_point) {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        
        if (!latest_lidar_cloud_ || latest_lidar_cloud_->empty()) {
            return -1.0; // No LiDAR data available
        }
        
        // Use KD-tree for efficient nearest neighbor search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(latest_lidar_cloud_);
        
        pcl::PointXYZ search_point;
        search_point.x = target_point.x;
        search_point.y = target_point.y;
        search_point.z = target_point.z;
        
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;
        
        // Search within 0.5m radius
        float radius = 0.5f;
        
        if (kdtree.radiusSearch(search_point, radius, point_idx_radius_search, point_radius_squared_distance) > 0) {
            // Return distance to nearest point
            return sqrt(point_radius_squared_distance[0]);
        }
        
        return -1.0; // No points found within radius
    }

    void publish_fused_poses() {
        std::lock_guard<std::mutex> lock(detections_mutex_);
        
        if (recent_detections_.empty()) {
            return;
        }
        
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = "odom";
        
        // Clean old detections (older than 1 second)
        auto current_time = this->get_clock()->now();
        auto it = recent_detections_.begin();
        while (it != recent_detections_.end()) {
            auto time_diff = current_time - it->second.last_seen;
            if (time_diff.seconds() > 1.0) {
                it = recent_detections_.erase(it);
            } else {
                pose_array.poses.push_back(process_marker_detection(it->second));
                ++it;
            }
        }
        
        if (!pose_array.poses.empty()) {
            // Publish standard PoseArray
            poses_publisher_->publish(pose_array);
            
            // Create and publish custom ArucoMarkers message
            rovers_interfaces::msg::ArucoMarkers aruco_msg;
            aruco_msg.header = pose_array.header;
            
            // Fill marker_ids and poses from recent detections
            for (const auto& [marker_id, detection] : recent_detections_) {
                auto time_diff = current_time - detection.last_seen;
                if (time_diff.seconds() <= 1.0) {  // Only include recent detections
                    aruco_msg.marker_ids.push_back(marker_id);
                    aruco_msg.poses.push_back(process_marker_detection(detection));
                }
            }
            
            aruco_markers_publisher_->publish(aruco_msg);
            
            // Publish visualization markers
            publish_visualization_markers(aruco_msg);
            
            // Reduced logging frequency
            static uint64_t publish_log_counter = 0;
            if (++publish_log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Published %zu ArUco markers", pose_array.poses.size());
            }
        }
    }

    double marker_size_;
    std::string aruco_dict_name_;
    std::string lidar_topic_;
    std::vector<std::string> camera_topics_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_publisher_;
    rclcpp::Publisher<rovers_interfaces::msg::ArucoMarkers>::SharedPtr aruco_markers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_viz_publisher_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // LiDAR data storage
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_lidar_cloud_;
    rclcpp::Time latest_lidar_timestamp_;
    std::mutex lidar_mutex_;
    
    // Detection storage and fusion
    std::map<int, MarkerDetection> recent_detections_;  // marker_id -> detection
    std::mutex detections_mutex_;
    
    // Pose filtering
    std::map<int, PoseFilter> pose_filters_;  // marker_id -> filter
    std::mutex filters_mutex_;
    
    // Status tracking
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::atomic<uint64_t> image_count_{0};
    std::atomic<uint64_t> lidar_count_{0};
    std::atomic<uint64_t> images_without_markers_{0};
    
    void publish_visualization_markers(const rovers_interfaces::msg::ArucoMarkers& aruco_msg) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Clear existing markers first
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = aruco_msg.header;
        clear_marker.ns = "aruco_tags";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        for (size_t i = 0; i < aruco_msg.marker_ids.size(); ++i) {
            // Create cube marker for each ArUco tag
            visualization_msgs::msg::Marker marker;
            marker.header = aruco_msg.header;
            marker.ns = "aruco_tags";
            marker.id = aruco_msg.marker_ids[i];
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set pose
            marker.pose = aruco_msg.poses[i];
            
            // Set scale (marker size) - make much bigger for visibility
            marker.scale.x = marker_size_ * 4.0;  // 4x larger (20cm instead of 5cm)
            marker.scale.y = marker_size_ * 4.0;  // 4x larger
            marker.scale.z = 0.05; // Thicker cube for better visibility
            
            // Set color (bright green for visibility)
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            // Set lifetime
            marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            
            marker_array.markers.push_back(marker);
            
            // Create text marker for ID
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = aruco_msg.header;
            text_marker.ns = "aruco_text";
            text_marker.id = aruco_msg.marker_ids[i];
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text above the bigger marker
            text_marker.pose = aruco_msg.poses[i];
            text_marker.pose.position.z += 0.15;  // Higher above the bigger marker
            
            // Set text content with padding for better visibility
            text_marker.text = "ID:" + std::to_string(aruco_msg.marker_ids[i]);
            
            // Set scale (text size) - much bigger
            text_marker.scale.z = 0.2;  // 2x bigger text
            
            // Set color (white text)
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // Set lifetime
            text_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            
            marker_array.markers.push_back(text_marker);
        }
        
        marker_viz_publisher_->publish(marker_array);
    }
    
    void print_status() {
        RCLCPP_INFO(this->get_logger(), "Status Report:");
        RCLCPP_INFO(this->get_logger(), "  - Images processed: %lu", image_count_.load());
        RCLCPP_INFO(this->get_logger(), "  - LiDAR clouds received: %lu", lidar_count_.load());
        RCLCPP_INFO(this->get_logger(), "  - Images without markers: %lu", images_without_markers_.load());
        RCLCPP_INFO(this->get_logger(), "  - Active marker detections: %zu", recent_detections_.size());
        
        if (image_count_ == 0) {
            RCLCPP_WARN(this->get_logger(), "No images received yet - check camera topics!");
        }
        if (lidar_count_ == 0) {
            RCLCPP_WARN(this->get_logger(), "No LiDAR data received yet - check LiDAR topic!");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoTagDetector>());
    rclcpp::shutdown();
    return 0;
}

