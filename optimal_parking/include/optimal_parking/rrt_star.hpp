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
    RRTStar(std::vector<Obstacle> obstacles,
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
    bool check_collision(const Eigen::Vector3d& state);
    bool check_path_collision(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    [[nodiscard]] Eigen::Vector3d step(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    static std::shared_ptr<Node> get_nearest_node(const std::vector<std::shared_ptr<Node>>& nodes, const Eigen::Vector3d& point);
    [[nodiscard]] std::vector<std::shared_ptr<Node>> find_nearby_nodes(const std::vector<std::shared_ptr<Node>>& nodes,
                                                                       const Eigen::Vector3d& point) const;
    std::vector<Eigen::Vector3d> make_path(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const size_t& path_length);

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
    static std::vector<Eigen::Vector3d> resample_path(const std::vector<Eigen::Vector3d>& path, size_t target_length);
};

};  // namespace optimal_parking

#endif  // RRT_STAR_HPP