#include "optimal_parking/system/system_model.hpp"

#include <unsupported/Eigen/MatrixFunctions>
#include <yaml-cpp/yaml.h>

namespace optimal_parking {
SystemModel::SystemModel(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);

    dt_ = config["mpc_dt"].as<double>();
    wheel_radius_ = config["wheel_radius"].as<double>();
    car_length_ = config["car_length"].as<double>();
    car_width_ = config["car_width"].as<double>();
}
SystemModel::SystemModel(const double& dt, const double& wheel_radius, const double& car_length, const double& car_width)
    : dt_(dt), wheel_radius_(wheel_radius), car_length_(car_length), car_width_(car_width) {}

SystemModel::SystemModel(const SystemModel& other)
    : dt_(other.dt_), wheel_radius_(other.wheel_radius_), car_length_(other.car_length_), car_width_(other.car_width_) {}

void SystemModel::initialize(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);

    dt_ = config["mpc_dt"].as<double>();
    wheel_radius_ = config["wheel_radius"].as<double>();
    car_length_ = config["car_length"].as<double>();
    car_width_ = config["car_width"].as<double>();
}
void SystemModel::initialize(const double& dt, const double& wheel_radius, const double& car_length, const double& car_width) {
    dt_ = dt;
    wheel_radius_ = wheel_radius;
    car_length_ = car_length;
    car_width_ = car_width;
}

Eigen::Vector<double, 5> SystemModel::f(const SystemState& state, const SystemInput& input) const {
    Eigen::Vector<double, 5> state_dot;
    // clang-format off
    state_dot <<   state.velocity() * std::cos(state.yaw()), 
                   state.velocity() * std::sin(state.yaw()), 
                   state.velocity() / car_length_ * std::tan(state.delta()),
                   input.acceleration(), 
                   input.steering_rate();
    // clang-format on
    return state_dot;
}

ModelMatrices SystemModel::getSystemJacobian(const SystemState& state, const SystemInput& input) {
    Eigen::Vector<double, 5> state_dot = f(state, input);
    // clang-format off
    Ac_ <<  0, 0, -state.velocity() * std::sin(state.yaw()),                  std::cos(state.yaw()),                                                                                     0,
            0, 0,  state.velocity() * std::cos(state.yaw()),                  std::sin(state.yaw()),                                                                                     0,
            0, 0,                                         0,  std::tan(state.delta()) / car_length_,  state.velocity() / (car_length_ * std::cos(state.delta()) * std::cos(state.delta())),
            0, 0,                                         0,                                      0,                                                                                     0, 
            0, 0,                                         0,                                      0,                                                                                     0;
    // clang-format on

    // clang-format off
	Bc_ <<  0, 0,
            0, 0,
            0, 0,
            1, 0,
            0, 1;
    // clang-format on
    gc_ = state_dot - Ac_ * state - Bc_ * input;

    Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> continuous_system_matrix = Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1>::Zero();

    continuous_system_matrix.block<5, 5>(0, 0) = Ac_;
    continuous_system_matrix.block<5, 2>(0, 5) = Bc_;
    continuous_system_matrix.block<5, 1>(0, 5 + 2) = gc_;
    continuous_system_matrix = continuous_system_matrix * dt_;
    const Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> discrete_system_matrix = continuous_system_matrix.exp();

    Ad_ = discrete_system_matrix.block<5, 5>(0, 0);
    Bd_ = discrete_system_matrix.block<5, 2>(0, 5);
    gd_ = discrete_system_matrix.block<5, 1>(0, 5 + 2);

    return ModelMatrices{Ad_, Bd_, gd_};
}
}  // namespace optimal_parking