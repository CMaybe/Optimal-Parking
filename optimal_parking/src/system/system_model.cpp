#include "optimal_parking/system/system_model.hpp"

#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

#include "optimal_parking/config.hpp"
namespace optimal_parking {
SystemModel::SystemModel(const std::string& path) {
    const PlannerConfig config = load_planner_config(path);
    vehicle_length_ = config.vehicle_length;
    vehicle_width_ = config.vehicle_width;
}
SystemModel::SystemModel(double vehicle_length, double vehicle_width)
    : vehicle_length_(vehicle_length), vehicle_width_(vehicle_width) {}
void SystemModel::initialize(const std::string& path) {
    const PlannerConfig config = load_planner_config(path);
    vehicle_length_ = config.vehicle_length;
    vehicle_width_ = config.vehicle_width;
}
void SystemModel::initialize(double vehicle_length, double vehicle_width) {
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

ModelMatrices SystemModel::get_system_jacobian(const SystemState& state, const SystemInput& input, const double& dt) const {
    Eigen::Vector<double, 5> state_dot = f(state, input);

    Eigen::Matrix<double, 5, 5> ac;
    Eigen::Matrix<double, 5, 5> ad;
    Eigen::Matrix<double, 5, 2> bc;
    Eigen::Matrix<double, 5, 2> bd;
    Eigen::Matrix<double, 5, 1> gc;
    Eigen::Matrix<double, 5, 1> gd;

    // clang-format off
        ac <<  0, 0, -state.velocity() * std::sin(state.yaw()),                     std::cos(state.yaw()),                                                                                        0,
            0, 0,  state.velocity() * std::cos(state.yaw()),                     std::sin(state.yaw()),                                                                                        0,
           0, 0,                                         0, std::tan(state.delta()) / vehicle_length_, state.velocity() / (vehicle_length_ * std::cos(state.delta()) * std::cos(state.delta())),
            0, 0,                                         0,                                         0,                                                                                        0,
            0, 0,                                         0,                                         0,                                                                                        0;
    // clang-format on

    // clang-format off
    bc <<  0, 0,
           0, 0,
           0, 0,
           1, 0,
           0, 1;
    // clang-format on
    gc = state_dot - ac * state - bc * input;

    Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> continuous_system_matrix = Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1>::Zero();

    continuous_system_matrix.block<5, 5>(0, 0) = ac;
    continuous_system_matrix.block<5, 2>(0, 5) = bc;
    continuous_system_matrix.block<5, 1>(0, 5 + 2) = gc;
    continuous_system_matrix = continuous_system_matrix * dt;
    const Eigen::Matrix<double, 5 + 2 + 1, 5 + 2 + 1> discrete_system_matrix = continuous_system_matrix.exp();

    ad = discrete_system_matrix.block<5, 5>(0, 0);
    bd = discrete_system_matrix.block<5, 2>(0, 5);
    gd = discrete_system_matrix.block<5, 1>(0, 5 + 2);

    return ModelMatrices{ad, bd, gd};
}
}  // namespace optimal_parking