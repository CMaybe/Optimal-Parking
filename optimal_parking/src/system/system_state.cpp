#include "optimal_parking/system/system_state.hpp"

namespace optimal_parking {
SystemState::SystemState() : x_(0), y_(0), yaw_(0), velocity_(0), delta_(0) { state_vector_.setZero(); }
SystemState::SystemState(double x, double y, double yaw, double velocity, double delta)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), delta_(delta) {
    state_vector_ << x, y, yaw, velocity, delta;
}

SystemState::SystemState(const SystemState& other)
    : x_(other.x_), y_(other.y_), yaw_(other.yaw_), velocity_(other.velocity_), delta_(other.delta_) {
    state_vector_ << x_, y_, yaw_, velocity_, delta_;
}

SystemState::SystemState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state)
    : x_(system_state[0]), y_(system_state[1]), yaw_(system_state[2]), velocity_(system_state[3]), delta_(system_state[4]) {
    state_vector_ = system_state;
}

SystemState& SystemState::operator=(const SystemState& other) {
    if (this == &other) {
        return *this;
    }
    x_ = other.x_;
    y_ = other.y_;
    yaw_ = other.yaw_;
    velocity_ = other.velocity_;
    delta_ = other.delta_;
    state_vector_ = other.state_vector_;
    return *this;
}
SystemState& SystemState::operator=(const Eigen::Vector<double, 5>& system_state) {
    state_vector_ = system_state;
    x_ = system_state[0];
    y_ = system_state[1];
    yaw_ = system_state[2];
    velocity_ = system_state[3];
    delta_ = system_state[4];
    return *this;
}

void SystemState::update_state(double x, double y, double yaw, double velocity, double delta) {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    velocity_ = velocity;
    delta_ = delta;

    state_vector_ << x_, y_, yaw_, velocity_, delta_;
}

void SystemState::update_state(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state) {
    state_vector_ = system_state;
    x_ = system_state[0];
    y_ = system_state[1];
    yaw_ = system_state[2];
    velocity_ = system_state[3];
    delta_ = system_state[4];
}

}  // namespace optimal_parking
