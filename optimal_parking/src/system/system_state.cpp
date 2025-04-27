#include "optimal_parking/system/system_state.hpp"

namespace optimal_parking {
SystemState::SystemState() : x_(0), y_(0), yaw_(0), velocity_(0), delta_(0) { system_state_.setZero(); }
SystemState::SystemState(const double& x, const double& y, const double& yaw, const double& velocity, const double& delta)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), delta_(delta) {
    system_state_ << x, y, yaw, velocity, delta;
}

SystemState::SystemState(const SystemState& other)
    : x_(other.x_), y_(other.y_), yaw_(other.yaw_), velocity_(other.velocity_), delta_(other.delta_) {
    system_state_ << x_, y_, yaw_, velocity_, delta_;
}

SystemState::SystemState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state)
    : x_(system_state[0]), y_(system_state[1]), yaw_(system_state[2]), velocity_(system_state[3]), delta_(system_state[4]) {
    system_state_ = system_state;
}

SystemState& SystemState::operator=(const SystemState& other) {
    if (this == &other) return *this;
    x_ = other.x_;
    y_ = other.y_;
    yaw_ = other.yaw_;
    velocity_ = other.velocity_;
    delta_ = other.delta_;
    system_state_ = other.system_state_;
    return *this;
}
SystemState& SystemState::operator=(const Eigen::Vector<double, 5>& system_state) {
    system_state_ = system_state;
    x_ = system_state[0];
    y_ = system_state[1];
    yaw_ = system_state[2];
    velocity_ = system_state[3];
    delta_ = system_state[4];
    return *this;
}

void SystemState::updateState(const double& x, const double& y, const double& yaw, const double& velocity, const double& delta) {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    velocity_ = velocity;
    delta_ = delta;

    system_state_ << x_, y_, yaw_, velocity_, delta_;
}

void SystemState::updateState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state) {
    system_state_ = system_state;
    x_ = system_state[0];
    y_ = system_state[1];
    yaw_ = system_state[2];
    velocity_ = system_state[3];
    delta_ = system_state[4];
}

}  // namespace optimal_parking
