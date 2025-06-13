#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace optimal_parking {

struct QPData {
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
};

struct ModelMatrices {
    Eigen::Matrix<double, 5, 5> Ad;
    Eigen::Matrix<double, 5, 2> Bd;
    Eigen::Matrix<double, 5, 1> gd;
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
    std::vector<std::shared_ptr<Node>> children;
    double cost;
};

};  // namespace optimal_parking