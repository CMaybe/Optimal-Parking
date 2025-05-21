#include "optimal_parking/rrt_star.hpp"

namespace optimal_parking {
RRTStar::RRTStar(const std::vector<Obstacle>& obstacles,
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
                 const double& vehicle_width)
    : obstacles_(obstacles)
    , goal_radius_(goal_radius)
    , goal_bias_(goal_bias)
    , step_dist_(step_dist)
    , rewire_radius_(rewire_radius)
    , max_iterations_(max_iterations)
    , vehicle_length_(vehicle_length)
    , vehicle_width_(vehicle_width)
    , gen_(rd_())
    , x_dist_(map_x_min, map_x_max)
    , y_dist_(map_y_min, map_y_max)
    , theta_dist_(-M_PI, M_PI)
    , bias_dist_(0.0, 1.0) {}

bool RRTStar::checkCollision(const Eigen::Vector3d& state) {
    Eigen::Vector2d pos = state.head<2>();
    double theta = state(2);
    Eigen::Matrix2d vehicle_rotate;
    vehicle_rotate << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    for (const auto& obs : obstacles_) {
        Eigen::Vector2d rel_pos = pos - obs.center;
        Eigen::Matrix2d obs_rotate;
        obs_rotate << std::cos(obs.yaw), std::sin(obs.yaw), -std::sin(obs.yaw), std::cos(obs.yaw);
        rel_pos = obs_rotate * rel_pos;
        if (std::abs(rel_pos(0)) < (obs.length / 2 + vehicle_length_ / 2) &&
            std::abs(rel_pos(1)) < (obs.width / 2 + vehicle_width_ / 2)) {
            return true;
        }
    }
    return false;
}

bool RRTStar::checkPathCollision(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    const int num_steps = 10;
    for (int i = 1; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        Eigen::Vector3d interp_state = from + t * (to - from);
        if (checkCollision(interp_state)) {
            return false;
        }
    }
    return true;
}

std::shared_ptr<Node> RRTStar::getNearestNode(const std::vector<std::shared_ptr<Node>>& nodes, const Eigen::Vector3d& point) {
    std::shared_ptr<Node> nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& node : nodes) {
        double dist = (node->state - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

Eigen::Vector3d RRTStar::step(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    double dist = (to - from).norm();
    if (dist < step_dist_) {
        return to;
    }
    return from + (to - from) * (step_dist_ / dist);
}

std::vector<std::shared_ptr<Node>> RRTStar::findNearbyNodes(const std::vector<std::shared_ptr<Node>>& nodes,
                                                            const Eigen::Vector3d& point) {
    std::vector<std::shared_ptr<Node>> nearby;
    for (const auto& node : nodes) {
        if ((node->state - point).norm() < rewire_radius_) {
            nearby.push_back(node);
        }
    }
    return nearby;
}

std::vector<Eigen::Vector3d> RRTStar::makePath(const Eigen::Vector3d& start,
                                               const Eigen::Vector3d& goal,
                                               const size_t& path_length) {
    std::vector<std::shared_ptr<Node>> nodes;
    std::vector<Eigen::Vector3d> path;
    auto start_node = std::make_shared<Node>();
    start_node->state = start;
    start_node->cost = 0.0;
    nodes.push_back(start_node);

    for (int i = 0; i < max_iterations_; ++i) {
        Eigen::Vector3d rand_point;
        if (bias_dist_(gen_) < goal_bias_) {
            rand_point = goal;
        } else {
            rand_point = Eigen::Vector3d(x_dist_(gen_), y_dist_(gen_), theta_dist_(gen_));
        }

        auto nearest_node = getNearestNode(nodes, rand_point);
        auto new_state = step(nearest_node->state, rand_point);
        if (!checkCollision(new_state) && checkPathCollision(nearest_node->state, new_state)) {
            auto new_node = std::make_shared<Node>();
            new_node->state = new_state;
            new_node->cost = nearest_node->cost + (new_state - nearest_node->state).norm();
            new_node->parent = nearest_node;
            auto nearby_nodes = findNearbyNodes(nodes, new_state);
            for (auto& near_node : nearby_nodes) {
                double new_cost = near_node->cost + (new_state - near_node->state).norm();
                if (new_cost < new_node->cost && checkPathCollision(near_node->state, new_state)) {
                    new_node->parent = near_node;
                    new_node->cost = new_cost;
                }
            }
            nodes.push_back(new_node);
            for (auto& near_node : nearby_nodes) {
                double new_cost = new_node->cost + (near_node->state - new_state).norm();
                if (new_cost < near_node->cost && checkPathCollision(new_state, near_node->state)) {
                    near_node->parent = new_node;
                    near_node->cost = new_cost;
                }
            }

            if ((new_state - goal).norm() < goal_radius_) {
                auto goal_node = std::make_shared<Node>();
                goal_node->state = goal;
                goal_node->cost = new_node->cost + (goal - new_state).norm();
                goal_node->parent = new_node;
                nodes.push_back(goal_node);

                auto current = goal_node;
                while (current != nullptr) {
                    path.push_back(current->state);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return resamplePath(path, path_length);
            }
        }
    }
    path.clear();
    return path;
}

std::vector<Eigen::Vector3d> RRTStar::resamplePath(const std::vector<Eigen::Vector3d>& path, size_t target_length) {
    std::vector<Eigen::Vector3d> resampled_path;
    if (path.size() < 2 || target_length == 0) return path;

    double total_distance = 0.0;
    std::vector<double> distances(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        total_distance += (path[i] - path[i - 1]).norm();
        distances[i] = total_distance;
    }
    resampled_path.push_back(path[0]);
    double step = total_distance / (target_length - 1);
    for (size_t i = 1; i < target_length - 1; ++i) {
        double target_dist = i * step;
        for (size_t j = 1; j < path.size(); ++j) {
            if (distances[j] >= target_dist) {
                double t = (target_dist - distances[j - 1]) / (distances[j] - distances[j - 1]);
                Eigen::Vector3d interp_state = path[j - 1] + t * (path[j] - path[j - 1]);
                resampled_path.push_back(interp_state);
                break;
            }
        }
    }
    resampled_path.push_back(path.back());

    return resampled_path;
}
}  // namespace optimal_parking