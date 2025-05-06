#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

using namespace std;

// Vertex representing a single parameter (x)
class VertexX : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override {
        _estimate = 0.0;
    }

    void oplusImpl(const double* update) override {
        _estimate += update[0];
    }

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

// Edge that encourages x to be close to 10
class EdgeX : public g2o::BaseUnaryEdge<1, double, VertexX> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override {
        const VertexX* v = static_cast<const VertexX*>(_vertices[0]);
        _error[0] = v->estimate() - _measurement;
    }

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

// ROS 2 Node wrapper
class G2ONode : public rclcpp::Node {
public:
    G2ONode() : Node("g2o_node") {
        run_optimization();
    }

private:
    void run_optimization() {
        RCLCPP_INFO(this->get_logger(), "Starting G2O optimization...");

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);

        auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
        auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
        optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver)));

        VertexX* v = new VertexX();
        v->setId(0);
        v->setEstimate(0.0);
        optimizer.addVertex(v);

        EdgeX* e = new EdgeX();
        e->setId(0);
        e->setVertex(0, v);
        e->setMeasurement(10.0);
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        optimizer.addEdge(e);

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        RCLCPP_INFO(this->get_logger(), "Optimized x = %f", v->estimate());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<G2ONode>());
    rclcpp::shutdown();
    return 0;
}
