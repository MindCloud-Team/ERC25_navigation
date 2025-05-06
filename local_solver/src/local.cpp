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

class TebPlanner : public rclcpp::Node {
public:
    using Pose2D = Eigen::Vector3d; // x, y, theta

    TebPlanner() : Node("teb_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Parameters matching original TEB
        declare_parameter("weight_obstacle", 50.0);
        declare_parameter("weight_kinematics_forward_drive", 100.0);
        declare_parameter("weight_kinematics_turning_radius", 50.0);
        declare_parameter("weight_time_optimal", 10.0);
        declare_parameter("weight_shortest_path", 5.0);
        declare_parameter("weight_global_plan", 20.0);
        declare_parameter("max_vel_x", 0.5);
        declare_parameter("max_vel_x_backwards", 0.2);
        declare_parameter("max_vel_theta", 1.0);
        declare_parameter("acc_lim_x", 0.5);
        declare_parameter("acc_lim_theta", 0.5);
        declare_parameter("min_turning_radius", 0.3);
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

        // Subscribers
        sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
            "/global_plan", 10, std::bind(&TebPlanner::pathCallback, this, std::placeholders::_1));
        sub_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap", 10, std::bind(&TebPlanner::costmapCallback, this, std::placeholders::_1));

        // Publisher
        pub_trajectory_ = create_publisher<nav_msgs::msg::Path>("/optimized_trajectory", 10);
        
        setupOptimizer();
    }

private:
    // ROS2 members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trajectory_;
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
        
        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& os) const override { return true; }
    };

    // 2. Obstacle Edge
    class ObstacleEdge : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void computeError() override {
            _error[0] = std::exp(-(_measurement - 0.1) / 0.2);
        }
        
        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& os) const override { return true; }
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
        
        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& os) const override { return true; }
        
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
        
        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& os) const override { return true; }
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
        
        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& os) const override { return true; }
    };

    // ========== OPTIMIZATION CORE ==========
    void setupOptimizer() {
        using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
        auto linearSolver = std::make_unique<g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>>();
        auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
        optimizer_ = std::make_unique<g2o::SparseOptimizer>();
        optimizer_->setAlgorithm(solver);
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_costmap_ = msg;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!latest_costmap_) {
            RCLCPP_WARN(get_logger(), "Waiting for costmap before optimizing.");
            return;
        }

        // Convert path to Eigen format
        std::vector<Pose2D> global_plan;
        for (const auto& pose : msg->poses) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double yaw = tf2::getYaw(pose.pose.orientation);
            global_plan.emplace_back(x, y, yaw);
        }

        // Optimize trajectory
        auto optimized = optimizePath(global_plan, *latest_costmap_);

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
    }

    std::vector<Pose2D> optimizePath(
        const std::vector<Pose2D>& global_plan,
        const nav_msgs::msg::OccupancyGrid& costmap
    ) {
        optimizer_->clear();

        // 1. Add vertices (poses)
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* v_pose = new g2o::VertexSE2();
            v_pose->setId(i);
            v_pose->setEstimate(g2o::SE2(global_plan[i][0], global_plan[i][1], global_plan[i][2]));
            optimizer_->addVertex(v_pose);
        }

        // 2. Add edges
        for (size_t i = 0; i < global_plan.size() - 1; ++i) {
            // Time optimal edge
            auto* edge_time = new TimeOptimalEdge();
            edge_time->setVertex(0, optimizer_->vertex(i));
            edge_time->setVertex(1, optimizer_->vertex(i+1));
            edge_time->setMeasurement(0.1);
            edge_time->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_time_optimal);
            optimizer_->addEdge(edge_time);

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

            // Shortest path edge
            auto* edge_short = new ShortestPathEdge();
            edge_short->setVertex(0, optimizer_->vertex(i));
            edge_short->setVertex(1, optimizer_->vertex(i+1));
            edge_short->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_shortest_path);
            optimizer_->addEdge(edge_short);
        }

        // Global plan edges
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* edge_global = new GlobalPlanEdge();
            edge_global->setVertex(0, optimizer_->vertex(i));
            edge_global->setMeasurement(global_plan[i].head<2>());
            edge_global->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_global_plan);
            optimizer_->addEdge(edge_global);
        }

        // Obstacle edges
        for (size_t i = 0; i < global_plan.size(); ++i) {
            double cost = getObstacleCost(global_plan[i], costmap);
            auto* edge_obs = new ObstacleEdge();
            edge_obs->setVertex(0, optimizer_->vertex(i));
            edge_obs->setMeasurement(cost);
            edge_obs->setInformation(Eigen::Matrix<double,1,1>::Identity() * config_.weight_obstacle);
            optimizer_->addEdge(edge_obs);
        }

        // Optimize
        optimizer_->initializeOptimization();
        optimizer_->optimize(100);

        // Extract optimized trajectory
        std::vector<Pose2D> result;
        for (size_t i = 0; i < global_plan.size(); ++i) {
            auto* v = static_cast<g2o::VertexSE2*>(optimizer_->vertex(i));
            result.emplace_back(
                v->estimate().translation().x(),
                v->estimate().translation().y(),
                v->estimate().rotation().angle()
            );
        }
        return result;
    }

    double getObstacleCost(const Pose2D& pose, const nav_msgs::msg::OccupancyGrid& costmap) {
        int x = static_cast<int>((pose[0] - costmap.info.origin.position.x) / costmap.info.resolution);
        int y = static_cast<int>((pose[1] - costmap.info.origin.position.y) / costmap.info.resolution);
        int idx = y * costmap.info.width + x;

        if (idx >= 0 && idx < static_cast<int>(costmap.data.size())) {
            return std::min(1.0, costmap.data[idx] / 100.0 + 0.1);
        }
        return 0.0;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TebPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}