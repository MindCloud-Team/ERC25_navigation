#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

// Add these three lines with your other PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <Eigen/Dense>

#include <deque>
#include <mutex>

using namespace gtsam;
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class LIOSAMNode : public rclcpp::Node
{
public:
    LIOSAMNode() : Node("lio_sam_node")
    {
        // Initialize parameters
        initializeParams();
        
        // Initialize GTSAM
        initializeGTSAM();
        
        // Initialize subscribers and publishers
        initializeSubscribers();
        initializePublishers();
        
        // Initialize point cloud processing
        initializePointCloudProcessing();
        
        RCLCPP_INFO(this->get_logger(), "LIO-SAM node initialized");
    }

private:
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    rclcpp::Time last_imu_time_;
    bool first_imu_received_ = false;

    bool is_stationary_;

    // Scan Context parameters
    const int PC_NUM_RING = 20; // Number of rings (rows) in the descriptor
    const int PC_NUM_SECTOR = 60; // Number of sectors (columns) in the descriptor
    const double PC_MAX_RADIUS = 80.0; // Max radius of the point cloud to consider

    // Database of keyframe descriptors
    std::vector<Eigen::MatrixXd> scan_context_database_;

    // Keyframe poses at the time of descriptor creation
    std::vector<Pose3> keyframe_poses_database_;
      
    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // GTSAM variables
    NonlinearFactorGraph graph_;
    Values initial_estimate_;
    ISAM2 isam_;
    Values isam_current_estimate_;
    
    // IMU preintegration
    std::shared_ptr<PreintegratedImuMeasurements> imu_preintegrated_;
    std::deque<sensor_msgs::msg::Imu> imu_queue_;
    std::mutex imu_mutex_;
    
    // Point cloud processing
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::CropBox<pcl::PointXYZ> crop_filter_;
    
    // State variables
    int key_frame_count_;
    Pose3 prev_pose_;
    Vector3 prev_vel_;
    imuBias::ConstantBias prev_bias_;
    NavState prev_state_;
    bool system_initialized_;
    
    // Costmap
    nav_msgs::msg::OccupancyGrid costmap_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> keyframe_clouds_;
    
    // Parameters
    double lidar_min_range_, lidar_max_range_;
    double voxel_size_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    int optimization_frequency_;
    bool enable_ground_removal_;
    double ground_distance_threshold_;
    
    void initializeParams()
    {
        this->declare_parameter("lidar_min_range", 1.0);
        this->declare_parameter("lidar_max_range", 100.0);
        this->declare_parameter("voxel_size", 0.1);
        this->declare_parameter("keyframe_distance_threshold", 1.0);
        this->declare_parameter("keyframe_angle_threshold", 0.2);
        this->declare_parameter("optimization_frequency", 10);
        this->declare_parameter("enable_ground_removal", false);  // Disabled by default
        this->declare_parameter("ground_distance_threshold", 0.2);

        this->declare_parameter("frames.map_frame", "map");
        this->declare_parameter("frames.odom_frame", "odom");
        this->declare_parameter("frames.base_link_frame", "base_link");

        this->declare_parameter("imu.accelerometer_noise_density", 0.01);
        this->declare_parameter("imu.gyroscope_noise_density", 0.001);
        
        lidar_min_range_ = this->get_parameter("lidar_min_range").as_double();
        lidar_max_range_ = this->get_parameter("lidar_max_range").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        keyframe_distance_threshold_ = this->get_parameter("keyframe_distance_threshold").as_double();
        keyframe_angle_threshold_ = this->get_parameter("keyframe_angle_threshold").as_double();
        optimization_frequency_ = this->get_parameter("optimization_frequency").as_int();
        enable_ground_removal_ = this->get_parameter("enable_ground_removal").as_bool();
        ground_distance_threshold_ = this->get_parameter("ground_distance_threshold").as_double();
        std::string map_frame_id = this->get_parameter("frames.map_frame").as_string();
        double gyro_noise = this->get_parameter("imu.gyroscope_noise_density").as_double();
    }
    
    void initializeGTSAM()
    {
        // ISAM2 parameters
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam_ = ISAM2(parameters);
        
        // Initialize state
        key_frame_count_ = 0;
        system_initialized_ = false;
        
        // IMU noise model (adjust these based on your IMU specs)
        auto p = PreintegrationParams::MakeSharedU(9.81);
        p->accelerometerCovariance = I_3x3 * pow(0.0565, 2);
        p->gyroscopeCovariance = I_3x3 * pow(0.004, 2);
        p->integrationCovariance = I_3x3 * pow(0.0, 2);
        // Note: Bias covariance parameters may not be available in this GTSAM version
        // They are set internally by GTSAM
        
        imu_preintegrated_ = std::make_shared<PreintegratedImuMeasurements>(p, imuBias::ConstantBias());
    }
    
    void initializeSubscribers()
    {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/velodyne_points", 10,
            std::bind(&LIOSAMNode::lidarCallback, this, std::placeholders::_1));
            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            std::bind(&LIOSAMNode::imuCallback, this, std::placeholders::_1));
    }
    
    void initializePublishers()
    {
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    Eigen::MatrixXd makeScanContext(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        Eigen::MatrixXd desc = Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
        
        for (const auto& point : cloud->points) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance >= PC_MAX_RADIUS) continue;

            // atan2 returns values from -pi to pi
            float angle = std::atan2(point.y, point.x);

            int ring_idx = std::floor(distance / (PC_MAX_RADIUS / PC_NUM_RING));
            int sector_idx = std::floor((angle + M_PI) / (2.0 * M_PI / PC_NUM_SECTOR));
            
            if (ring_idx >= 0 && ring_idx < PC_NUM_RING && sector_idx >= 0 && sector_idx < PC_NUM_SECTOR) {
                // Use the maximum height of a point in a cell as the cell's value
                if (point.z > desc(ring_idx, sector_idx)) {
                    desc(ring_idx, sector_idx) = point.z;
                }
            }
        }
        return desc;
    }

    void detectLoopClosure(const Pose3& current_pose, int current_keyframe_id)
    {
        // Don't try to find loops until we have enough keyframes in the database
        if (keyframe_clouds_.size() < 20) return;

        // Create the Scan Context for the current keyframe
        Eigen::MatrixXd current_sc = makeScanContext(keyframe_clouds_.back());
        
        double min_dist = 10000.0;
        int matched_id = -1;

        // Search the database for the best match
        // We search backwards and ignore recent keyframes to avoid matching with the immediate past
        for (int i = 0; i < (int)scan_context_database_.size() - 15; ++i) {
            const auto& past_sc = scan_context_database_[i];
            
            // Find the best rotational alignment
            int best_shift = 0;
            double min_col_dist = 10000.0;
            for (int shift = 0; shift < PC_NUM_SECTOR; ++shift) {
                Eigen::MatrixXd shifted_sc(PC_NUM_RING, PC_NUM_SECTOR);
                for (int j = 0; j < PC_NUM_SECTOR; ++j) {
                    shifted_sc.col(j) = past_sc.col((j + shift) % PC_NUM_SECTOR);
                }
                double dist = calculateDistance(current_sc, shifted_sc);
                if (dist < min_col_dist) {
                    min_col_dist = dist;
                    best_shift = shift;
                }
            }
            
            if (min_col_dist < min_dist) {
                min_dist = min_col_dist;
                matched_id = i;
            }
        }

        // If we found a good enough match, add a loop closure factor
        // Thresholds for distance and ID difference are crucial and may need tuning.
        const double LOOP_CLOSURE_THRESHOLD = 0.6; 
        if (min_dist < LOOP_CLOSURE_THRESHOLD && matched_id != -1) {
            RCLCPP_INFO(this->get_logger(), ">>>> LOOP CLOSURE DETECTED between keyframe %d and %d! <<<<", current_keyframe_id, matched_id);
            
            // Add a BetweenFactor to the graph connecting the current pose and the matched pose
            auto noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());
            
            // We add the factor to the main graph, not the temporary one.
            // It will be processed in the next big optimization.
            graph_.add(BetweenFactor<Pose3>(X(current_keyframe_id), X(matched_id), Pose3::Identity(), noise));

            // After adding a loop closure, it's a good idea to trigger a full graph optimization
            optimizeGraph(true); // Force an optimization
        }
    }
    
    double calculateDistance(const Eigen::MatrixXd& desc1, const Eigen::MatrixXd& desc2)
    {
        double sum_dist = 0.0;
        for (int i = 0; i < desc1.cols(); ++i) {
            sum_dist += (desc1.col(i) - desc2.col(i)).norm();
        }
        return sum_dist / desc1.cols();
    }
    
    void initializePointCloudProcessing()
    {
        // Voxel grid filter
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        
        // Crop box filter
        crop_filter_.setMin(Eigen::Vector4f(-lidar_max_range_, -lidar_max_range_, -3.0, 1.0));
        crop_filter_.setMax(Eigen::Vector4f(lidar_max_range_, lidar_max_range_, 1.0, 1.0));
        
        // Initialize costmap
        costmap_.header.frame_id = "map";
        costmap_.info.resolution = 0.1; // 10cm resolution
        costmap_.info.width = 2000;     // 200m x 200m map
        costmap_.info.height = 2000;
        costmap_.info.origin.position.x = -100.0;
        costmap_.info.origin.position.y = -100.0;
        costmap_.data.resize(costmap_.info.width * costmap_.info.height, -1); // Unknown
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        // Check for stationarity
        double linear_accel_norm = std::sqrt(
            msg->linear_acceleration.x * msg->linear_acceleration.x +
            msg->linear_acceleration.y * msg->linear_acceleration.y +
            msg->linear_acceleration.z * msg->linear_acceleration.z
        );
        double angular_vel_norm = std::sqrt(
            msg->angular_velocity.x * msg->angular_velocity.x +
            msg->angular_velocity.y * msg->angular_velocity.y +
            msg->angular_velocity.z * msg->angular_velocity.z
        );

        // Thresholds to determine if the robot is stationary. Tune these if needed.
        // If linear acceleration is very close to G and angular velocity is very low.
        if (linear_accel_norm < 10.0 && linear_accel_norm > 9.6 && angular_vel_norm < 0.05) {
            is_stationary_ = true;
        } else {
            is_stationary_ = false;
        }
        
        imu_queue_.push_back(*msg);
        if (!system_initialized_) return;

        // Calculate dt dynamically
        double dt = 0.0;
        if (!first_imu_received_) {
            last_imu_time_ = msg->header.stamp;
            first_imu_received_ = true;
            return;
        }
        dt = (rclcpp::Time(msg->header.stamp) - last_imu_time_).seconds();
        last_imu_time_ = msg->header.stamp;

        if (dt <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid IMU dt: %f", dt);
            return;
        }

        Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Vector3 gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        
        imu_preintegrated_->integrateMeasurement(acc, gyro, dt);
    }
    
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // If the IMU detects we are stationary, do not process this Lidar scan.
        // Just publish the last known pose to keep TF alive and return.
        if (is_stationary_) {
            // Get the latest known pose and publish it
            if (key_frame_count_ > 0) {
                 Pose3 last_pose = prev_state_.pose();
                 publishOdometry(last_pose, msg->header.stamp);
            }
            return; 
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = processPointCloud(raw_cloud);
        
        if (!system_initialized_)
        {
            initializeSystem(processed_cloud, msg->header.stamp);
            return;
        }
        
        Pose3 current_pose = performScanMatching(processed_cloud);
        
        if (isKeyFrame(current_pose))
        {
            addKeyFrame(current_pose, processed_cloud, msg->header.stamp);
            optimizeGraph(); // This now calls the version without the 'force' parameter
            updateCostmap();
        }
        
        publishOdometry(current_pose, msg->header.stamp);
        publishProcessedCloud(processed_cloud, msg->header.stamp);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // --- Step 1: Range filtering first to reduce processing ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points)
        {
            float range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (range >= lidar_min_range_ && range <= lidar_max_range_)
            {
                range_filtered->points.push_back(point);
            }
        }
        range_filtered->width = range_filtered->points.size();
        range_filtered->height = 1;
        
        // --- Step 2: Voxel Grid Downsampling ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(range_filtered);
        voxel_filter_.filter(*voxel_filtered);

        // --- Step 3: Optional Ground Removal (disabled by default) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud;
        if (enable_ground_removal_) {
            final_cloud = removeGround(voxel_filtered);
        } else {
            final_cloud = voxel_filtered;
        }
        
        return final_cloud;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ground_distance_threshold_); // More conservative threshold
        seg.setMaxIterations(100); // Limit iterations
        seg.setInputCloud(cloud);
        
        // Try to find a plane
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0 || inliers->indices.size() < cloud->points.size() * 0.1) {
            // Failed to find a significant plane, or plane is too small - use original cloud
            RCLCPP_DEBUG(this->get_logger(), "No significant ground plane found, keeping all points");
            groundless_cloud = cloud;
        } else {
            // Check if the detected plane is likely to be ground (approximately horizontal)
            double normal_z = std::abs(coefficients->values[2]); // Z component of normal
            if (normal_z > 0.8) { // Normal should be mostly vertical for ground plane
                // Extract everything EXCEPT the inliers (the ground plane)
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);
                extract.setNegative(true); // Remove the plane
                extract.filter(*groundless_cloud);
                RCLCPP_DEBUG(this->get_logger(), "Ground plane removed: %zu points removed", inliers->indices.size());
            } else {
                // Detected plane is not horizontal enough to be ground
                groundless_cloud = cloud;
                RCLCPP_DEBUG(this->get_logger(), "Detected plane not horizontal enough (normal_z=%.2f), keeping all points", normal_z);
            }
        }
        
        return groundless_cloud;
    }
    
    void initializeSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Time /* timestamp */)
    {
        // Initialize first pose at origin with proper orientation
        // Make sure initial pose is level (no roll/pitch)
        prev_pose_ = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
        prev_vel_ = Vector3::Zero();
        prev_bias_ = imuBias::ConstantBias();
        
        prev_state_ = NavState(prev_pose_, prev_vel_); 
        
        // Add prior factors with appropriate noise
        auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        auto vel_noise = noiseModel::Isotropic::Sigma(3, 0.1);
        auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);
        
        graph_.add(PriorFactor<Pose3>(X(0), prev_pose_, pose_noise));
        graph_.add(PriorFactor<Vector3>(V(0), prev_vel_, vel_noise));
        graph_.add(PriorFactor<imuBias::ConstantBias>(B(0), prev_bias_, bias_noise));
        
        initial_estimate_.insert(X(0), prev_pose_);
        initial_estimate_.insert(V(0), prev_vel_);
        initial_estimate_.insert(B(0), prev_bias_);
        
        // Store first keyframe
        keyframe_clouds_.push_back(cloud);
        key_frame_count_ = 1;
        system_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "System initialized at origin");
    }
    
    // Helper function to convert Eigen::Matrix4f to gtsam::Pose3
    gtsam::Pose3 eigenMatrixToGtsamPose(const Eigen::Matrix4f& mat) {
        return gtsam::Pose3(gtsam::Rot3(mat.block<3, 3>(0, 0).cast<double>()),
                             gtsam::Point3(mat.block<3, 1>(0, 3).cast<double>()));
    }

    Pose3 performScanMatching(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud)
    {
        if (keyframe_clouds_.empty()) return prev_state_.pose();

        // Create a local map from recent keyframes
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>());
        int start_index = std::max(0, (int)keyframe_clouds_.size() - 5); // Use fewer keyframes for stability
        
        for (size_t i = start_index; i < keyframe_clouds_.size(); ++i) {
            Pose3 keyframe_pose;
            if (isam_current_estimate_.exists(X(i))) {
                keyframe_pose = isam_current_estimate_.at<Pose3>(X(i));
            } else if (initial_estimate_.exists(X(i))) {
                keyframe_pose = initial_estimate_.at<Pose3>(X(i));
            } else {
                continue;
            }
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            Eigen::Matrix4f transform = keyframe_pose.matrix().cast<float>();
            pcl::transformPointCloud(*keyframe_clouds_[i], *transformed_cloud, transform);
            *local_map += *transformed_cloud;
        }

        // Downsample local map
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_local_map(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_filter_.setInputCloud(local_map);
        voxel_filter_.filter(*downsampled_local_map);

        // ICP configuration
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputTarget(downsampled_local_map);
        icp.setMaximumIterations(30); // Reduced iterations
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(0.1); // More lenient fitness
        icp.setMaxCorrespondenceDistance(1.5); // Reduced for more stable matching

        // Get IMU prediction for initial guess
        NavState predicted_state = imu_preintegrated_->predict(prev_state_, prev_bias_);
        Eigen::Matrix4f initial_guess = predicted_state.pose().matrix().cast<float>();

        // Transform current cloud to predicted position
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*current_cloud, *transformed_current_cloud, initial_guess);

        icp.setInputSource(transformed_current_cloud);
        
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);

        if (icp.hasConverged() && icp.getFitnessScore() < 1.0) // Check fitness score
        {
            Eigen::Matrix4f correction_transform = icp.getFinalTransformation();
            Pose3 final_pose = eigenMatrixToGtsamPose(correction_transform * initial_guess);
            
            // Sanity check: don't allow huge jumps
            double translation_change = (final_pose.translation() - prev_state_.pose().translation()).norm();
            if (translation_change < 5.0) { // Max 5m jump
                return final_pose;
            }
        }

        // Fallback to IMU prediction
        RCLCPP_DEBUG(this->get_logger(), "ICP failed or unreliable, using IMU prediction");
        return predicted_state.pose();
    }
    
    bool isKeyFrame(const Pose3& current_pose)
    {
        if (key_frame_count_ == 0) return true;
        
        Pose3 last_keyframe_pose;
        if (isam_current_estimate_.exists(X(key_frame_count_ - 1))) {
            last_keyframe_pose = isam_current_estimate_.at<Pose3>(X(key_frame_count_ - 1));
        } else {
            last_keyframe_pose = initial_estimate_.at<Pose3>(X(key_frame_count_ - 1));
        }

        double distance = (current_pose.translation() - last_keyframe_pose.translation()).norm();
        double angle = Rot3::Logmap(last_keyframe_pose.rotation().inverse() * current_pose.rotation()).norm();

        return (distance > keyframe_distance_threshold_ || angle > keyframe_angle_threshold_);
    }
    
    void addKeyFrame(const Pose3& pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Time /* timestamp */)
    {
        RCLCPP_INFO(this->get_logger(), "Adding keyframe %d at position (%.2f, %.2f, %.2f)", 
                    key_frame_count_, pose.x(), pose.y(), pose.z());

        // Add IMU factor if we have preintegrated measurements
        if (imu_preintegrated_->deltaTij() > 0)
        {
            auto imu_factor = ImuFactor(
                X(key_frame_count_ - 1), V(key_frame_count_ - 1),
                X(key_frame_count_),   V(key_frame_count_),
                B(key_frame_count_ - 1), *imu_preintegrated_);
            graph_.add(imu_factor);
            
            auto bias_factor = BetweenFactor<imuBias::ConstantBias>(
                B(key_frame_count_ - 1), B(key_frame_count_),
                imuBias::ConstantBias(),
                noiseModel::Diagonal::Sigmas(sqrt(imu_preintegrated_->deltaTij()) * 
                    (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()));
            graph_.add(bias_factor);
        }
        
        // Predict the new state using IMU preintegration
        NavState predicted_state = imu_preintegrated_->predict(prev_state_, prev_bias_);
        
        // Add initial estimates to the graph
        initial_estimate_.insert(X(key_frame_count_), pose); 
        initial_estimate_.insert(V(key_frame_count_), predicted_state.v());
        initial_estimate_.insert(B(key_frame_count_), prev_bias_);
        
        keyframe_clouds_.push_back(cloud);
        
        // Add Scan Context to database for loop closure
        scan_context_database_.push_back(makeScanContext(cloud));
        keyframe_poses_database_.push_back(pose);

        // Detect loop closures
        detectLoopClosure(pose, key_frame_count_);
        
        // Reset and update state for the next interval
        imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);
        prev_state_ = NavState(pose, predicted_state.v()); 
        key_frame_count_++;
    }
    
    void optimizeGraph(bool force_optimization = false)
    {
        if (force_optimization || (key_frame_count_ > 0 && key_frame_count_ % optimization_frequency_ == 0))
        {
            RCLCPP_INFO(this->get_logger(), "Optimizing graph with %d keyframes...", key_frame_count_);
            
            try {
                isam_.update(graph_, initial_estimate_);
                isam_.update();
                
                isam_current_estimate_ = isam_.calculateEstimate();
                
                if (isam_current_estimate_.exists(X(key_frame_count_-1)))
                {
                    Pose3 latest_pose = isam_current_estimate_.at<Pose3>(X(key_frame_count_-1));
                    Vector3 latest_vel = isam_current_estimate_.at<Vector3>(V(key_frame_count_-1));
                    imuBias::ConstantBias latest_bias = isam_current_estimate_.at<imuBias::ConstantBias>(B(key_frame_count_-1));
                    prev_state_ = NavState(latest_pose, latest_vel);
                    prev_bias_ = latest_bias;
                }
                
                graph_.resize(0);
                initial_estimate_.clear();
                
                RCLCPP_INFO(this->get_logger(), "Graph optimization complete.");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Graph optimization failed: %s", e.what());
            }
        }
    }
    
    // Helper function for ray-tracing using Bresenham's line algorithm
    void raytraceLine(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int e2;

        while (true) {
            // Don't mark the endpoint as free, only the cells leading up to it
            if (x0 == x1 && y0 == y1) break;

            // Check bounds
            if (x0 >= 0 && x0 < (int)costmap_.info.width && y0 >= 0 && y0 < (int)costmap_.info.height) {
                int index = y0 * costmap_.info.width + x0;
                if (costmap_.data[index] != 100) { // Don't overwrite occupied cells
                    costmap_.data[index] = 0; // Mark as free
                }
            }

            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    void updateCostmap()
    {
        // Initialize the map to all -1 (unknown)
        std::fill(costmap_.data.begin(), costmap_.data.end(), -1);

        std::set<int> all_occupied_indices;

        // FIRST PASS: Collect all occupied cell indices from keyframes
        for (size_t i = 0; i < keyframe_clouds_.size(); ++i) {
            
            Pose3 keyframe_pose;
            bool pose_found = false;

            if (isam_current_estimate_.exists(X(i))) {
                keyframe_pose = isam_current_estimate_.at<Pose3>(X(i));
                pose_found = true;
            } else if (initial_estimate_.exists(X(i))) {
                keyframe_pose = initial_estimate_.at<Pose3>(X(i));
                pose_found = true;
            }

            if (!pose_found) continue;
            
            // Transform points to world coordinates
            for (const auto& point : keyframe_clouds_[i]->points) {
                gtsam::Point3 world_point = keyframe_pose.transformFrom(gtsam::Point3(point.x, point.y, point.z));
                
                int map_x = (world_point.x() - costmap_.info.origin.position.x) / costmap_.info.resolution;
                int map_y = (world_point.y() - costmap_.info.origin.position.y) / costmap_.info.resolution;

                if (map_x >= 0 && map_x < (int)costmap_.info.width && 
                    map_y >= 0 && map_y < (int)costmap_.info.height) {
                    int index = map_y * costmap_.info.width + map_x;
                    all_occupied_indices.insert(index);
                }
            }
        }

        // SECOND PASS: Perform ray-tracing to clear free space
        for (size_t i = 0; i < keyframe_clouds_.size(); ++i) {
            
            Pose3 keyframe_pose;
            bool pose_found = false;

            if (isam_current_estimate_.exists(X(i))) {
                keyframe_pose = isam_current_estimate_.at<Pose3>(X(i));
                pose_found = true;
            } else if (initial_estimate_.exists(X(i))) {
                keyframe_pose = initial_estimate_.at<Pose3>(X(i));
                pose_found = true;
            }

            if (!pose_found) continue;

            int sensor_map_x = (keyframe_pose.x() - costmap_.info.origin.position.x) / costmap_.info.resolution;
            int sensor_map_y = (keyframe_pose.y() - costmap_.info.origin.position.y) / costmap_.info.resolution;

            // Ray trace to a subset of points to avoid excessive computation
            for (size_t j = 0; j < keyframe_clouds_[i]->points.size(); j += 3) { // Sample every 3rd point
                const auto& point = keyframe_clouds_[i]->points[j];
                gtsam::Point3 world_point = keyframe_pose.transformFrom(gtsam::Point3(point.x, point.y, point.z));
                
                int point_map_x = (world_point.x() - costmap_.info.origin.position.x) / costmap_.info.resolution;
                int point_map_y = (world_point.y() - costmap_.info.origin.position.y) / costmap_.info.resolution;

                raytraceLine(sensor_map_x, sensor_map_y, point_map_x, point_map_y);
            }
        }

        // FINAL PASS: Mark all occupied cells as 100
        for (int index : all_occupied_indices) {
            costmap_.data[index] = 100;
        }
        
        costmap_.header.stamp = this->get_clock()->now();
        costmap_pub_->publish(costmap_);
    }
    
    void publishOdometry(const Pose3& pose, rclcpp::Time timestamp)
    {
        // Publish the Odometry Message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        
        auto translation = pose.translation();
        auto rotation = pose.rotation().toQuaternion();
        
        odom.pose.pose.position.x = translation.x();
        odom.pose.pose.position.y = translation.y();
        odom.pose.pose.position.z = translation.z();
        odom.pose.pose.orientation.w = rotation.w();
        odom.pose.pose.orientation.x = rotation.x();
        odom.pose.pose.orientation.y = rotation.y();
        odom.pose.pose.orientation.z = rotation.z();
        
        // Add covariance information
        odom.pose.covariance[0] = 0.1;  // x
        odom.pose.covariance[7] = 0.1;  // y  
        odom.pose.covariance[14] = 0.1; // z
        odom.pose.covariance[21] = 0.1; // roll
        odom.pose.covariance[28] = 0.1; // pitch
        odom.pose.covariance[35] = 0.1; // yaw
        
        odom_pub_->publish(odom);

        // Broadcast the TF Transform
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = timestamp;
        tf_stamped.header.frame_id = "map";
        tf_stamped.child_frame_id = "base_link";
        tf_stamped.transform.translation.x = translation.x();
        tf_stamped.transform.translation.y = translation.y();
        tf_stamped.transform.translation.z = translation.z();
        tf_stamped.transform.rotation.w = rotation.w();
        tf_stamped.transform.rotation.x = rotation.x();
        tf_stamped.transform.rotation.y = rotation.y();
        tf_stamped.transform.rotation.z = rotation.z();

        tf_broadcaster_->sendTransform(tf_stamped);
    }
    
    void publishProcessedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Time timestamp)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = "base_link";
        cloud_pub_->publish(cloud_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIOSAMNode>());
    rclcpp::shutdown();
    return 0;
}
