// teb_final_planner.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h> 
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  // Added for tf2::getYaw()
#include <iostream>
#include <cmath>
#include <chrono>

class TebPlanner : public rclcpp::Node {
public:
    using Pose2D = Eigen::Vector3d; // x, y, theta

    TebPlanner() : Node("teb_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        RCLCPP_INFO(get_logger(), "Initializing TEB Planner...");
        
        // Parameters optimized for speed
        declare_parameter("weight_obstacle", 30.0);  // Reduced for faster convergence
        declare_parameter("weight_kinematics_forward_drive", 50.0);  // Reduced
        declare_parameter("weight_kinematics_turning_radius", 30.0);  // Reduced
        declare_parameter("weight_time_optimal", 15.0);  // Increased for faster paths
        declare_parameter("weight_shortest_path", 10.0);  // Increased
        declare_parameter("weight_global_plan", 10.0);  // Reduced for more flexibility
        declare_parameter("max_vel_x", 1.2);  // Increased max velocity
        declare_parameter("max_vel_x_backwards", 0.3);
        declare_parameter("max_vel_theta", 1.5);  // Increased turning speed
        declare_parameter("acc_lim_x", 1.0);  // Increased acceleration
        declare_parameter("acc_lim_theta", 1.0);  // Increased angular acceleration
        declare_parameter("min_turning_radius", 0.2);  // Reduced for tighter turns
        declare_parameter("footprint_model.type", "circular");
        declare_parameter("footprint_model.radius", 0.3);

        // Get parameters
        get_parameter("weight_obstacle", config_.weight_obstacle);
        get_parameter("weight_kinematics_forward_drive", config_.weight_kinematics_forward_drive);
        get_parameter("weight_kinematics_turning_radius", config_.weight_kinematics_turning_radius);
        get_parameter("weight_time_optimal", config_.weight_time_optimal);
        get_parameter("weight_shortest_path", config_.weight_shortest_path);
        get_parameter("weight_global_plan", config_.weight_global_plan);
        get_parameter("max_vel_x", config_.max_vel_x);
        get_parameter("max_vel_x_backwards", config_.max_vel_x_backwards);
        get_parameter("max_vel_theta", config_.max_vel_theta);
        get_parameter("acc_lim_x", config_.acc_lim_x);
        get_parameter("acc_lim_theta", config_.acc_lim_theta);
        get_parameter("min_turning_radius", config_.min_turning_radius);

        // Debug: Log loaded parameters
        RCLCPP_INFO(get_logger(), "=== TEB Configuration ===");
        RCLCPP_INFO(get_logger(), "Weight obstacle: %.2f", config_.weight_obstacle);
        RCLCPP_INFO(get_logger(), "Weight kinematics forward: %.2f", config_.weight_kinematics_forward_drive);
        RCLCPP_INFO(get_logger(), "Weight kinematics turning: %.2f", config_.weight_kinematics_turning_radius);
        RCLCPP_INFO(get_logger(), "Weight time optimal: %.2f", config_.weight_time_optimal);
        RCLCPP_INFO(get_logger(), "Weight shortest path: %.2f", config_.weight_shortest_path);
        RCLCPP_INFO(get_logger(), "Weight global plan: %.2f", config_.weight_global_plan);
        RCLCPP_INFO(get_logger(), "Max velocity X: %.2f m/s", config_.max_vel_x);
        RCLCPP_INFO(get_logger(), "Max velocity theta: %.2f rad/s", config_.max_vel_theta);
        RCLCPP_INFO(get_logger(), "Min turning radius: %.2f m", config_.min_turning_radius);
        RCLCPP_INFO(get_logger(), "========================");

        // Subscribers - try common nav2 topic names
        sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
            "/global_planner", 10, std::bind(&TebPlanner::pathCallback, this, std::placeholders::_1));
        
        // Try multiple costmap topic names
        sub_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", 10, std::bind(&TebPlanner::costmapCallback, this, std::placeholders::_1));
        sub_costmap_alt_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&TebPlanner::costmapCallback, this, std::placeholders::_1));
        sub_costmap_alt2_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap", 10, std::bind(&TebPlanner::costmapCallback, this, std::placeholders::_1));

        // Publisher
        pub_trajectory_ = create_publisher<nav_msgs::msg::Path>("/optimized_trajectory", 10);
        
        RCLCPP_INFO(get_logger(), "Subscribers and publishers created");
        RCLCPP_INFO(get_logger(), "Subscribed to: /plan");
        RCLCPP_INFO(get_logger(), "Trying costmap topics: /local_costmap/costmap, /costmap, /local_costmap");
        RCLCPP_INFO(get_logger(), "Publishing to: /optimized_trajectory");
        
        // Create a timer to check for available topics
        topic_discovery_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TebPlanner::checkAvailableTopics, this));
        
        setupOptimizer();
        RCLCPP_INFO(get_logger(), "TEB Planner initialization complete!");
    }

private:
    // ROS2 members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_alt_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_alt2_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trajectory_;
    rclcpp::TimerBase::SharedPtr topic_discovery_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    // TEB Config (matches original)
    struct Config {
        double weight_obstacle;
        double weight_kinematics_forward_drive;
        double weight_kinematics_turning_radius;
        double weight_time_optimal;
        double weight_shortest_path;
        double weight_global_plan;
        double max_vel_x;
        double max_vel_x_backwards;
        double max_vel_theta;
        double acc_lim_x;
        double acc_lim_theta;
        double min_turning_radius;
    } config_;

    // G2O Optimizer
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;

    // ========== CORE TEB EDGES ==========

    // 1. Time Optimal Edge
    class TimeOptimalEdge : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void computeError() override {
            const auto* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
            const auto* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
            double dist = (v2->estimate().translation() - v1->estimate().translation()).norm();
            _error[0] = dist / _measurement;
        }
        
        virtual bool read(std::istream& is) override { (void)is; return true; }
        virtual bool write(std::ostream& os) const override { (void)os; return true; }
    };

    // 2. Obstacle Edge
    class ObstacleEdge : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void computeError() override {
            _error[0] = std::exp(-(_measurement - 0.1) / 0.2);
        }
        
        virtual bool read(std::istream& is) override { (void)is; return true; }
        virtual bool write(std::ostream& os) const override { (void)os; return true; }
    };

    // 3. Kinematics Edge
    class KinematicsEdge : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE2, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        KinematicsEdge(double max_vel_x, double max_vel_theta, double min_radius) 
            : max_vel_x_(max_vel_x), max_vel_theta_(max_vel_theta), min_radius_(min_radius) {}
        
        void computeError() override {
            const auto* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
            const auto* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
            
            double dt = _measurement[0];
            Eigen::Vector2d delta_pos = v2->estimate().translation() - v1->estimate().translation();
            double delta_theta = g2o::normalize_theta(v2->estimate().rotation().angle() - 
                                                     v1->estimate().rotation().angle());
            
            double vel = delta_pos.norm() / dt;
            _error[0] = std::max(0.0, vel - max_vel_x_);
            
            double omega = delta_theta / dt;
            _error[1] = std::max(0.0, std::abs(omega) - max_vel_theta_);
            
            if (delta_pos.norm() > 0.01) {
                double radius = delta_pos.norm() / (2 * std::sin(delta_theta/2));
                _error[2] = std::max(0.0, 1.0/radius - 1.0/min_radius_);
            } else {
                _error[2] = 0.0;
            }
        }
        
        virtual bool read(std::istream& is) override { (void)is; return true; }
        virtual bool write(std::ostream& os) const override { (void)os; return true; }
        
    private:
        double max_vel_x_, max_vel_theta_, min_radius_;
    };

    // 4. Global Plan Edge
    class GlobalPlanEdge : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void computeError() override {
            const auto* v = static_cast<const g2o::VertexSE2*>(_vertices[0]);
            _error[0] = (v->estimate().translation() - _measurement).squaredNorm();
        }
        
        virtual bool read(std::istream& is) override { (void)is; return true; }
        virtual bool write(std::ostream& os) const override { (void)os; return true; }
    };

    // 5. Shortest Path Edge
    class ShortestPathEdge : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void computeError() override {
            const auto* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);
            const auto* v2 = static_cast<const g2o::VertexSE2*>(_vertices[1]);
            _error[0] = (v2->estimate().translation() - v1->estimate().translation()).norm();
        }
        
        virtual bool read(std::istream& is) override { (void)is; return true; }
        virtual bool write(std::ostream& os) const override { (void)os; return true; }
    };

    void checkAvailableTopics() {
        auto topics = get_topic_names_and_types();
        
        std::vector<std::string> path_topics;
        std::vector<std::string> costmap_topics;
        
        for (const auto& topic_info : topics) {
            const std::string& topic_name = topic_info.first;
            const std::vector<std::string>& topic_types = topic_info.second;
            
            // Look for nav_msgs/Path topics
            for (const auto& type : topic_types) {
                if (type == "nav_msgs/msg/Path") {
                    if (topic_name.find("path") != std::string::npos || 
                        topic_name.find("plan") != std::string::npos) {
                        path_topics.push_back(topic_name);
                    }
                } else if (type == "nav_msgs/msg/OccupancyGrid") {
                    if (topic_name.find("costmap") != std::string::npos) {
                        costmap_topics.push_back(topic_name);
                    }
                }
            }
        }
        
        if (!path_topics.empty()) {
            RCLCPP_INFO(get_logger(), "=== Available Path Topics ===");
            for (const auto& topic : path_topics) {
                RCLCPP_INFO(get_logger(), "  - %s", topic.c_str());
            }
        }
        
        if (!costmap_topics.empty()) {
            RCLCPP_INFO(get_logger(), "=== Available Costmap Topics ===");
            for (const auto& topic : costmap_topics) {
                RCLCPP_INFO(get_logger(), "  - %s", topic.c_str());
            }
        }
        
        if (!path_topics.empty() || !costmap_topics.empty()) {
            RCLCPP_INFO(get_logger(), "===============================");
            // Cancel the timer after first discovery
            topic_discovery_timer_->cancel();
        } else {
            RCLCPP_WARN(get_logger(), "No relevant topics found! Is your navigation stack running?");
        }
    }

    std::vector<Pose2D> downsamplePath(const std::vector<Pose2D>& path, double max_distance = 0.1) {
        if (path.size() <= 2) return path;
        
        std::vector<Pose2D> downsampled;
        downsampled.reserve(path.size() / 2);  // Reserve memory for efficiency
        downsampled.push_back(path[0]); // Always keep first point
        
        // Adaptive downsampling - more aggressive for long paths
        double adaptive_distance = max_distance;
        if (path.size() > 200) adaptive_distance = max_distance * 2.0;
        else if (path.size() > 100) adaptive_distance = max_distance * 1.5;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double dist = (path[i].head<2>() - downsampled.back().head<2>()).norm();
            if (dist >= adaptive_distance) {
                downsampled.push_back(path[i]);
            }
        }
        
        downsampled.push_back(path.back()); // Always keep last point
        
        RCLCPP_INFO(get_logger(), "Downsampled path from %zu to %zu poses (%.1fcm spacing)", 
                   path.size(), downsampled.size(), adaptive_distance * 100);
        return downsampled;
    }

    bool isInsideCostmap(const Pose2D& pose, const nav_msgs::msg::OccupancyGrid& costmap) {
        int x = static_cast<int>((pose[0] - costmap.info.origin.position.x) / costmap.info.resolution);
        int y = static_cast<int>((pose[1] - costmap.info.origin.position.y) / costmap.info.resolution);
        
        return (x >= 0 && x < static_cast<int>(costmap.info.width) && 
                y >= 0 && y < static_cast<int>(costmap.info.height));
    }

    void setupOptimizer() {
        RCLCPP_DEBUG(get_logger(), "Setting up G2O optimizer...");
        try {
            using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
            auto linearSolver = std::make_unique<g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>>();
            auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
            auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
            optimizer_ = std::make_unique<g2o::SparseOptimizer>();
            optimizer_->setAlgorithm(solver);
            optimizer_->setVerbose(false);  // Disable verbose output for speed
            RCLCPP_DEBUG(get_logger(), "G2O optimizer setup complete");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to setup optimizer: %s", e.what());
            throw;
        }
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_DEBUG(get_logger(), "Received costmap: %dx%d, resolution: %.3f m/pixel", 
                    msg->info.width, msg->info.height, msg->info.resolution);
        RCLCPP_DEBUG(get_logger(), "Costmap origin: (%.2f, %.2f)", 
                    msg->info.origin.position.x, msg->info.origin.position.y);
        
        // Log costmap coverage area
        double max_x = msg->info.origin.position.x + msg->info.width * msg->info.resolution;
        double max_y = msg->info.origin.position.y + msg->info.height * msg->info.resolution;
        RCLCPP_DEBUG(get_logger(), "Costmap covers: (%.2f, %.2f) to (%.2f, %.2f)", 
                    msg->info.origin.position.x, msg->info.origin.position.y, max_x, max_y);
        
        latest_costmap_ = msg;
        
        // Log some costmap statistics
        if (!msg->data.empty()) {
            int occupied_cells = 0;
            int free_cells = 0;
            int unknown_cells = 0;
            
            for (const auto& cell : msg->data) {
                if (cell > 80) occupied_cells++;
                else if (cell >= 0) free_cells++;
                else unknown_cells++;
            }
            
            RCLCPP_DEBUG(get_logger(), "Costmap stats - Free: %d, Occupied: %d, Unknown: %d", 
                        free_cells, occupied_cells, unknown_cells);
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(get_logger(), "=== Path Optimization Started ===");
        RCLCPP_INFO(get_logger(), "Received global path with %zu poses", msg->poses.size());
        
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path, skipping optimization");
            return;
        }

        if (!latest_costmap_) {
            RCLCPP_WARN(get_logger(), "No costmap available, waiting for costmap before optimizing");
            return;
        }

        // Check costmap age
        auto costmap_age = (this->get_clock()->now() - latest_costmap_->header.stamp).seconds();
        if (costmap_age > 5.0) {
            RCLCPP_WARN(get_logger(), "Costmap is %.1f seconds old, optimization may be unreliable", costmap_age);
        }

        // Convert path to Eigen format
        std::vector<Pose2D> global_plan;
        RCLCPP_DEBUG(get_logger(), "Converting path poses to internal format...");
        
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double yaw = tf2::getYaw(pose.pose.orientation);
            global_plan.emplace_back(x, y, yaw);
            
            if (i == 0 || i == msg->poses.size() - 1) {
                RCLCPP_DEBUG(get_logger(), "Pose %zu: (%.2f, %.2f, %.2f rad)", i, x, y, yaw);
            }
        }

        // Check if path is mostly inside costmap
        int inside_count = 0;
        for (const auto& pose : global_plan) {
            if (isInsideCostmap(pose, *latest_costmap_)) {
                inside_count++;
            }
        }
        
        double inside_ratio = static_cast<double>(inside_count) / global_plan.size();
        RCLCPP_INFO(get_logger(), "Path coverage: %d/%zu poses (%.1f%%) inside costmap", 
                   inside_count, global_plan.size(), inside_ratio * 100);
        
        if (inside_ratio < 0.5) {
            RCLCPP_WARN(get_logger(), "Less than 50%% of path is inside costmap! Optimization may fail.");
        }

        // Aggressive downsampling for speed
        if (global_plan.size() > 50) {  // Reduced threshold
            RCLCPP_WARN(get_logger(), "Path has %zu poses, downsampling for performance", global_plan.size());
            global_plan = downsamplePath(global_plan, 0.1); // Increased to 10cm spacing
        }

        // Calculate path length
        double total_length = 0.0;
        for (size_t i = 1; i < global_plan.size(); ++i) {
            total_length += (global_plan[i].head<2>() - global_plan[i-1].head<2>()).norm();
        }
        RCLCPP_INFO(get_logger(), "Original path length: %.2f m", total_length);

        // Optimize trajectory
        RCLCPP_INFO(get_logger(), "Starting trajectory optimization...");
        auto optimized = optimizePath(global_plan, *latest_costmap_);

        if (optimized.empty()) {
            RCLCPP_ERROR(get_logger(), "Optimization failed, no trajectory generated");
            return;
        }

        // Calculate optimized path length
        double optimized_length = 0.0;
        for (size_t i = 1; i < optimized.size(); ++i) {
            optimized_length += (optimized[i].head<2>() - optimized[i-1].head<2>()).norm();
        }
        RCLCPP_INFO(get_logger(), "Optimized path length: %.2f m (change: %.2f m)", 
                   optimized_length, optimized_length - total_length);

        // Publish result
        nav_msgs::msg::Path path_msg;
        path_msg.header = msg->header;

        for (const auto& pose : optimized) {
            geometry_msgs::msg::PoseStamped p;
            p.header = msg->header;
            p.pose.position.x = pose[0];
            p.pose.position.y = pose[1];
            tf2::Quaternion q;
            q.setRPY(0, 0, pose[2]);
            p.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(p);
        }

        pub_trajectory_->publish(path_msg);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(get_logger(), "Published optimized trajectory with %zu poses", path_msg.poses.size());
        RCLCPP_INFO(get_logger(), "Total optimization time: %ld ms", duration.count());
        RCLCPP_INFO(get_logger(), "=== Path Optimization Complete ===");
    }

    std::vector<Pose2D> optimizePath(
        const std::vector<Pose2D>& global_plan,
        const nav_msgs::msg::OccupancyGrid& costmap
    ) {
        RCLCPP_DEBUG(get_logger(), "Clearing previous optimization problem...");
        optimizer_->clear();

        RCLCPP_DEBUG(get_logger(), "Adding %zu pose vertices to optimization problem...", global_plan.size());
        
        // 1. Add vertices (poses)
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* v_pose = new g2o::VertexSE2();
            v_pose->setId(i);
            v_pose->setEstimate(g2o::SE2(global_plan[i][0], global_plan[i][1], global_plan[i][2]));
            
            // Fix first and last poses
            if (i == 0 || i == global_plan.size() - 1) {
                v_pose->setFixed(true);
                RCLCPP_DEBUG(get_logger(), "Fixed vertex %zu at (%.2f, %.2f)", i, global_plan[i][0], global_plan[i][1]);
            }
            
            optimizer_->addVertex(v_pose);
        }

        int edge_count = 0;
        
        // 2. Add sequential edges
        RCLCPP_DEBUG(get_logger(), "Adding sequential edges between poses...");
        for (size_t i = 0; i < global_plan.size() - 1; ++i) {
            // Time optimal edge
            auto* edge_time = new TimeOptimalEdge();
            edge_time->setVertex(0, optimizer_->vertex(i));
            edge_time->setVertex(1, optimizer_->vertex(i+1));
            edge_time->setMeasurement(0.1);
            edge_time->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_time_optimal);
            optimizer_->addEdge(edge_time);
            edge_count++;

            // Kinematics edge
            auto* edge_kin = new KinematicsEdge(
                config_.max_vel_x, 
                config_.max_vel_theta,
                config_.min_turning_radius
            );
            edge_kin->setVertex(0, optimizer_->vertex(i));
            edge_kin->setVertex(1, optimizer_->vertex(i+1));
            edge_kin->setMeasurement(Eigen::Vector3d(0.1, 0, 0));
            edge_kin->setInformation(Eigen::Matrix<double,3,3>::Identity() * 
                Eigen::Vector3d(
                    config_.weight_kinematics_forward_drive,
                    config_.weight_kinematics_turning_radius,
                    config_.weight_kinematics_turning_radius
                ).asDiagonal());
            optimizer_->addEdge(edge_kin);
            edge_count++;

            // Shortest path edge
            auto* edge_short = new ShortestPathEdge();
            edge_short->setVertex(0, optimizer_->vertex(i));
            edge_short->setVertex(1, optimizer_->vertex(i+1));
            edge_short->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_shortest_path);
            optimizer_->addEdge(edge_short);
            edge_count++;
        }

        // Global plan edges
        RCLCPP_DEBUG(get_logger(), "Adding global plan constraint edges...");
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* edge_global = new GlobalPlanEdge();
            edge_global->setVertex(0, optimizer_->vertex(i));
            edge_global->setMeasurement(global_plan[i].head<2>());
            edge_global->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_global_plan);
            optimizer_->addEdge(edge_global);
            edge_count++;
        }

        // Obstacle edges
        RCLCPP_DEBUG(get_logger(), "Adding obstacle avoidance edges...");
        double total_obstacle_cost = 0.0;
        int high_cost_poses = 0;
        
        for (size_t i = 0; i < global_plan.size(); ++i) {
            double cost = getObstacleCost(global_plan[i], costmap);
            total_obstacle_cost += cost;
            if (cost > 0.5) high_cost_poses++;
            
            auto* edge_obs = new ObstacleEdge();
            edge_obs->setVertex(0, optimizer_->vertex(i));
            edge_obs->setMeasurement(cost);
            edge_obs->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_obstacle);
            optimizer_->addEdge(edge_obs);
            edge_count++;
        }

        RCLCPP_INFO(get_logger(), "Optimization problem setup complete:");
        RCLCPP_INFO(get_logger(), "  - Vertices: %zu", global_plan.size());
        RCLCPP_INFO(get_logger(), "  - Edges: %d", edge_count);
        RCLCPP_INFO(get_logger(), "  - Average obstacle cost: %.3f", total_obstacle_cost / global_plan.size());
        RCLCPP_INFO(get_logger(), "  - High cost poses: %d", high_cost_poses);

        // Optimize
        RCLCPP_INFO(get_logger(), "Running optimization (max 100 iterations)...");
        auto opt_start = std::chrono::high_resolution_clock::now();
        
        bool init_success = optimizer_->initializeOptimization();
        if (!init_success) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize optimization!");
            return {};
        }
        
        // Run optimization with safer parameters
        int iterations = 0;
        try {
            // Validate optimizer state before running
            if (optimizer_->vertices().size() == 0 || optimizer_->edges().size() == 0) {
                RCLCPP_ERROR(get_logger(), "Optimizer has no vertices or edges!");
                return {};
            }
            
            iterations = optimizer_->optimize(50);  // Conservative iteration count
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Optimization threw exception: %s", e.what());
            return {};
        }
        
        auto opt_end = std::chrono::high_resolution_clock::now();
        auto opt_duration = std::chrono::duration_cast<std::chrono::milliseconds>(opt_end - opt_start);
        
        if (opt_duration.count() > 1000) { // 1 second timeout (reduced from 5)
            RCLCPP_WARN(get_logger(), "Optimization took too long (%ld ms), may have failed", opt_duration.count());
        }
        
        RCLCPP_INFO(get_logger(), "Optimization completed in %d iterations (%ld ms)", 
                   iterations, opt_duration.count());

        if (iterations == 0) {
            RCLCPP_WARN(get_logger(), "Optimization completed 0 iterations - may have failed immediately");
        }

        // Extract optimized trajectory with safety checks
        RCLCPP_DEBUG(get_logger(), "Extracting optimized trajectory...");
        std::vector<Pose2D> result;
        result.reserve(global_plan.size());  // Pre-allocate memory
        
        double max_displacement = 0.0;
        double total_displacement = 0.0;
        
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* v = static_cast<g2o::VertexSE2*>(optimizer_->vertex(i));
            if (!v) {
                RCLCPP_ERROR(get_logger(), "Null vertex at index %zu!", i);
                return {};
            }
            
            // Validate the estimate before using it
            auto estimate = v->estimate();
            if (!std::isfinite(estimate.translation().x()) || 
                !std::isfinite(estimate.translation().y()) ||
                !std::isfinite(estimate.rotation().angle())) {
                RCLCPP_WARN(get_logger(), "Invalid pose at index %zu, using original", i);
                result.push_back(global_plan[i]);
                continue;
            }
            
            Pose2D optimized_pose(
                estimate.translation().x(),
                estimate.translation().y(),
                estimate.rotation().angle()
            );
            result.push_back(optimized_pose);
            
            // Calculate displacement from original
            double displacement = (optimized_pose.head<2>() - global_plan[i].head<2>()).norm();
            total_displacement += displacement;
            max_displacement = std::max(max_displacement, displacement);
        }

        RCLCPP_INFO(get_logger(), "Trajectory extraction complete:");
        RCLCPP_INFO(get_logger(), "  - Average displacement: %.3f m", total_displacement / global_plan.size());
        RCLCPP_INFO(get_logger(), "  - Maximum displacement: %.3f m", max_displacement);
        
        return result;
    }

    double getObstacleCost(const Pose2D& pose, const nav_msgs::msg::OccupancyGrid& costmap) {
        int x = static_cast<int>((pose[0] - costmap.info.origin.position.x) / costmap.info.resolution);
        int y = static_cast<int>((pose[1] - costmap.info.origin.position.y) / costmap.info.resolution);
        
        // Check bounds
        if (x < 0 || x >= static_cast<int>(costmap.info.width) || 
            y < 0 || y >= static_cast<int>(costmap.info.height)) {
            
            // Only log first few out-of-bounds poses to avoid spam
            static int out_of_bounds_count = 0;
            if (out_of_bounds_count < 5) {
                RCLCPP_DEBUG(get_logger(), "Pose (%.2f, %.2f) outside costmap bounds", pose[0], pose[1]);
                out_of_bounds_count++;
                if (out_of_bounds_count == 5) {
                    RCLCPP_DEBUG(get_logger(), "Further out-of-bounds messages suppressed...");
                }
            }
            return 0.5; // Medium cost for out-of-bounds (not too high to avoid breaking optimization)
        }
        
        int idx = y * costmap.info.width + x;

        if (idx >= 0 && idx < static_cast<int>(costmap.data.size())) {
            double cost = std::min(1.0, costmap.data[idx] / 100.0 + 0.1);
            
            // Log high-cost areas less frequently
            if (cost > 0.8) {
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                                    "High obstacle cost %.2f at pose (%.2f, %.2f)", cost, pose[0], pose[1]);
            }
            
            return cost;
        }
        
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                            "Invalid costmap index %d for pose (%.2f, %.2f)", idx, pose[0], pose[1]);
        return 0.5;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Set logging level for more detailed output
    auto ret = rcutils_logging_set_logger_level("teb_planner", RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        printf("Failed to set logger level\n");
    }
    
    auto node = std::make_shared<TebPlanner>();
    
    RCLCPP_INFO(node->get_logger(), "TEB Planner node started and ready!");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "TEB Planner node shutting down...");
    rclcpp::shutdown();
    return 0;
}

