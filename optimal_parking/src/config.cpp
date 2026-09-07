#include "optimal_parking/config.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace optimal_parking {
namespace {

template <int Size>
Eigen::Vector<double, Size> read_vector(const YAML::Node& node) {
    const auto values = node.as<std::vector<double>>();
    if (values.size() != Size) {
        throw std::invalid_argument("Configuration vector has an unexpected size");
    }
    const auto mapped_values = Eigen::Map<const Eigen::Vector<double, Size>>(values.data());
    return Eigen::Vector<double, Size>(mapped_values);
}

Obstacle read_obstacle(const YAML::Node& node) {
    Obstacle obstacle;
    obstacle.center = read_vector<2>(node["center"]);
    obstacle.length = node["length"].as<double>();
    obstacle.width = node["width"].as<double>();
    obstacle.yaw = node["yaw"].as<double>();
    return obstacle;
}

}  // namespace

PlannerConfig load_planner_config(const std::string& path) {
    const YAML::Node node = YAML::LoadFile(path);
    PlannerConfig config;

    config.vehicle_length = node["vehicle_length"].as<double>();
    config.vehicle_width = node["vehicle_width"].as<double>();
    config.trajectory_time = node["trajectory_time"].as<double>();
    config.ts = node["Ts"].as<double>();
    config.state_lower_bound = read_vector<5>(node["state_lowerbound"]);
    config.state_upper_bound = read_vector<5>(node["state_upperbound"]);
    config.input_lower_bound = read_vector<2>(node["input_lowerbound"]);
    config.input_upper_bound = read_vector<2>(node["input_upperbound"]);
    config.state_weight = read_vector<5>(node["state_weight"]);
    config.input_weight = read_vector<2>(node["input_weight"]);

    config.sqp_iterations = node["n_sqp"].as<int>();
    config.qp_iterations = node["qp_iteration"].as<int>();
    config.goal_penalty = node["rho_goal"].as<double>();
    config.obstacle_penalty = node["rho_obs"].as<double>();
    config.safety_margin = node["safety_margin"].as<double>();

    for (const auto& obstacle_node : node["obstacles"]) {
        config.obstacles.push_back(read_obstacle(obstacle_node));
    }

    config.map_x_min = node["map_x_min"].as<double>();
    config.map_x_max = node["map_x_max"].as<double>();
    config.map_y_min = node["map_y_min"].as<double>();
    config.map_y_max = node["map_y_max"].as<double>();
    config.goal_radius = node["goal_radius"].as<double>();
    config.goal_bias = node["goal_bias"].as<double>();
    config.step_distance = node["step_dist"].as<double>();
    config.rewire_radius = node["rewire_radius"].as<double>();
    config.rrt_iterations = node["max_iterations"].as<int>();

    return config;
}

}  // namespace optimal_parking
