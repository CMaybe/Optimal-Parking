#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "optimal_parking/types.hpp"

namespace optimal_parking {

struct PlannerConfig {
    double vehicle_length;
    double vehicle_width;
    double trajectory_time;
    double ts;

    Eigen::Vector<double, 5> state_lower_bound;
    Eigen::Vector<double, 5> state_upper_bound;
    Eigen::Vector<double, 2> input_lower_bound;
    Eigen::Vector<double, 2> input_upper_bound;
    Eigen::Vector<double, 5> state_weight;
    Eigen::Vector<double, 2> input_weight;

    int sqp_iterations;
    int qp_iterations;
    double goal_penalty;
    double obstacle_penalty;
    double safety_margin;

    std::vector<Obstacle> obstacles;

    double map_x_min;
    double map_x_max;
    double map_y_min;
    double map_y_max;
    double goal_radius;
    double goal_bias;
    double step_distance;
    double rewire_radius;
    int rrt_iterations;
};

[[nodiscard]] PlannerConfig load_planner_config(const std::string& path);

}  // namespace optimal_parking
