#include "optimal_parking/system/system_model.hpp"

#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <yaml-cpp/yaml.h>
namespace optimal_parking {
SystemModel::SystemModel(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);

    vehicle_length_ = config["vehicle_length"].as<double>();
    vehicle_width_ = config["vehicle_width"].as<double>();
}
SystemModel::SystemModel(const double& vehicle_length, const double& vehicle_width)
    : vehicle_length_(vehicle_length), vehicle_width_(vehicle_width) {}
SystemModel::SystemModel(const SystemModel& other)
    : vehicle_length_(other.vehicle_length_), vehicle_width_(other.vehicle_width_) {}

void SystemModel::initialize(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);

    vehicle_length_ = config["vehicle_length"].as<double>();
    vehicle_width_ = config["vehicle_width"].as<double>();
}
void SystemModel::initialize(const double& vehicle_length, const double& vehicle_width) {
    vehicle_length_ = vehicle_length;
    vehicle_width_ = vehicle_width;
}

Eigen::Vector<double, 5> SystemModel::f(const SystemState& state, const SystemInput& input) const {
    Eigen::Vector<double, 5> state_dot;
    // clang-format off
    state_dot <<   state.velocity() * std::cos(state.yaw()), 
                   state.velocity() * std::sin(state.yaw()), 
                   state.velocity() * std::tan(state.delta()) / vehicle_length_ ,
                   input.acceleration(), 
                   input.steering_rate();
    // clang-format on
    return state_dot;
}

ModelMatrices SystemModel::getSystemJacobian(const SystemState& state, const SystemInput& input, const double& dt) const {
    Eigen::Vector<double, 5> state_dot = f(state, input);

    Eigen::Matrix<double, 5, 5> Ac, Ad;
    Eigen::Matrix<double, 5, 2> Bc, Bd;
    Eigen::Matrix<double, 5, 1> gc, gd;

    // clang-format off
    Ac <<  0, 0, -state.velocity() * std::sin(state.yaw()),                     std::cos(state.yaw()),                                                                                        0, 
           0, 0,  state.velocity() * std::cos(state.yaw()),                     std::sin(state.yaw()),                                                                                        0, 
           0, 0,                                         0, std::tan(state.delta()) / vehicle_length_, state.velocity() / (vehicle_length_ * std::cos(state.delta()) * std::cos(state.delta())),
           0, 0,                                         0,                                         0,                                                                                        0, 
           0, 0,                                         0,                                         0,                                                                                        0;
    // clang-format on	

    // clang-format off
    Bc <<  0, 0,
           0, 0,
           0, 0,
           1, 0,
           0, 1;
    // clang-format on
    gc = state_dot - Ac * state - Bc * input;

    Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> continuous_system_matrix = Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1>::Zero();

    continuous_system_matrix.block<5, 5>(0, 0) = Ac;
    continuous_system_matrix.block<5, 2>(0, 5) = Bc;
    continuous_system_matrix.block<5, 1>(0, 5 + 2) = gc;
    continuous_system_matrix = continuous_system_matrix * dt;
    const Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> discrete_system_matrix = continuous_system_matrix.exp();

    Ad = discrete_system_matrix.block<5, 5>(0, 0);
    Bd = discrete_system_matrix.block<5, 2>(0, 5);
    gd = discrete_system_matrix.block<5, 1>(0, 5 + 2);

    return ModelMatrices{Ad, Bd, gd};
}
}  // namespace optimal_parking