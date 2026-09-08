#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace optimal_parking {

struct QPData {
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> constraint_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
};

struct ModelMatrices {
    Eigen::Matrix<double, 5, 5> discrete_a;
    Eigen::Matrix<double, 5, 2> discrete_b;
    Eigen::Matrix<double, 5, 1> discrete_g;
};

struct TrajectoryData {
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
    std::vector<double> velocity;
    std::vector<double> steering_angle;

    std::vector<double> acceleration;
    std::vector<double> steering_rate;
};

struct Obstacle {
    Eigen::Vector2d center;
    double length;
    double width;
    double yaw;
};

struct Node {
    Eigen::Vector3d state;
    std::shared_ptr<Node> parent;
    double cost;
};

}  // namespace optimal_parking