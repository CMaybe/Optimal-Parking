#include "optimal_parking/system/system_model.hpp"

#include <cmath>

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

Eigen::Vector<double, 5> SystemModel::evaluate_dynamics(const SystemState& state, const SystemInput& input) const {
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

ModelMatrices SystemModel::compute_discrete_linearization(const SystemState& state,
                                                          const SystemInput& input,
                                                          const double time_step) const {
    Eigen::Vector<double, 5> state_dot = evaluate_dynamics(state, input);

    Eigen::Matrix<double, 5, 5> continuous_a;
    Eigen::Matrix<double, 5, 5> discrete_a;
    Eigen::Matrix<double, 5, 2> continuous_b;
    Eigen::Matrix<double, 5, 2> discrete_b;
    Eigen::Matrix<double, 5, 1> continuous_g;
    Eigen::Matrix<double, 5, 1> discrete_g;

    // clang-format off
    continuous_a <<  0, 0, -state.velocity() * std::sin(state.yaw()),                     std::cos(state.yaw()),                                                                                        0,
           0, 0,  state.velocity() * std::cos(state.yaw()),                     std::sin(state.yaw()),                                                                                        0,
           0, 0,                                         0, std::tan(state.delta()) / vehicle_length_, state.velocity() / (vehicle_length_ * std::cos(state.delta()) * std::cos(state.delta())),
           0, 0,                                         0,                                         0,                                                                                        0,
           0, 0,                                         0,                                         0,                                                                                        0;
    // clang-format on

    // clang-format off
    continuous_b <<  0, 0,
           0, 0,
           0, 0,
           1, 0,
           0, 1;
    // clang-format on
    continuous_g = state_dot - continuous_a * state - continuous_b * input;

    const Eigen::Matrix<double, 5, 5> continuous_a_dt = continuous_a * time_step;
    const Eigen::Matrix<double, 5, 5> identity = Eigen::Matrix<double, 5, 5>::Identity();
    const Eigen::Matrix<double, 5, 5> identity_plus_half_a_dt = identity + 0.5 * continuous_a_dt;

    discrete_a = identity + continuous_a_dt + 0.5 * (continuous_a_dt * continuous_a_dt);
    discrete_b = identity_plus_half_a_dt * (continuous_b * time_step);
    discrete_g = identity_plus_half_a_dt * (continuous_g * time_step);

    return ModelMatrices{discrete_a, discrete_b, discrete_g};
}
}  // namespace optimal_parking