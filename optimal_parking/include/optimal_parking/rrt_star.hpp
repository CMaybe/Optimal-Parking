#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <vector>

#include "optimal_parking/types.hpp"

namespace optimal_parking {

class RRTStar {
public:
    RRTStar(const std::vector<Obstacle>& obstacles,
            const double& map_x_min,
            const double& map_x_max,
            const double& map_y_min,
            const double& map_y_max,
            const double& goal_radius,
            const double& goal_bias,
            const double& step_dist,
            const double& rewire_radius,
            const int& max_iterations,
            const double& vehicle_length,
            const double& vehicle_width);
    bool checkCollision(const Eigen::Vector3d& state);
    bool checkPathCollision(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    Eigen::Vector3d step(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    std::shared_ptr<Node> getNearestNode(const std::vector<std::shared_ptr<Node>>& nodes, const Eigen::Vector3d& point);
    std::vector<std::shared_ptr<Node>> findNearbyNodes(const std::vector<std::shared_ptr<Node>>& nodes,
                                                       const Eigen::Vector3d& point);
    std::vector<Eigen::Vector3d> makePath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const size_t& path_length);

private:
    std::vector<Obstacle> obstacles_;

    double goal_radius_, goal_bias_;
    double step_dist_;
    double rewire_radius_;
    int max_iterations_;

    double vehicle_length_, vehicle_width_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;
    std::uniform_real_distribution<> theta_dist_;
    std::uniform_real_distribution<> bias_dist_;
    std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d>& path, size_t path_length);
};

};  // namespace optimal_parking

#endif  // RRT_STAR_HPP