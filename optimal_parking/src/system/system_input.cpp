#include "optimal_parking/system/system_input.hpp"

namespace optimal_parking {

SystemInput::SystemInput(const double& acceleration, const double& steering_rate)
    : acceleration_(acceleration), steering_rate_(steering_rate) {
    system_input_ << acceleration, steering_rate;
}

SystemInput::SystemInput(const SystemInput& other) : acceleration_(other.acceleration_), steering_rate_(other.steering_rate_) {
    system_input_ = other.system_input_;
}

SystemInput::SystemInput(const Eigen::Ref<const Eigen::Vector2d>& system_input)
    : acceleration_(system_input[0]), steering_rate_(system_input[1]) {
    system_input_ = system_input;
}

SystemInput& SystemInput::operator=(const SystemInput& other) {
    if (this == &other) return *this;
    acceleration_ = other.acceleration_;
    steering_rate_ = other.steering_rate_;
    return *this;
}

void SystemInput::updateInput(const double& acceleration, const double& steering_rate) {
    acceleration_ = acceleration;
    steering_rate_ = steering_rate;
    system_input_ << acceleration, steering_rate;
}

void SystemInput::updateInput(const Eigen::Ref<const Eigen::Vector2d>& system_state) {
    acceleration_ = system_state[0];
    steering_rate_ = system_state[1];
    system_input_ = system_state;
}

}  // namespace optimal_parking
