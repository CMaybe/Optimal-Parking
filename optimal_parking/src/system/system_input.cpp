#include "optimal_parking/system/system_input.hpp"

namespace optimal_parking {

SystemInput::SystemInput() : acceleration_(0), steering_rate_(0) { input_vector_.setZero(); }

SystemInput::SystemInput(double acceleration, double steering_rate) : acceleration_(acceleration), steering_rate_(steering_rate) {
    input_vector_ << acceleration, steering_rate;
}

SystemInput::SystemInput(const SystemInput& other) : acceleration_(other.acceleration_), steering_rate_(other.steering_rate_) {
    input_vector_ = other.input_vector_;
}

SystemInput::SystemInput(const Eigen::Ref<const Eigen::Vector2d>& system_input)
    : acceleration_(system_input[0]), steering_rate_(system_input[1]) {
    input_vector_ = system_input;
}

SystemInput& SystemInput::operator=(const SystemInput& other) {
    if (this == &other) {
        return *this;
    }
    acceleration_ = other.acceleration_;
    steering_rate_ = other.steering_rate_;
    input_vector_ = other.input_vector_;
    return *this;
}

void SystemInput::update_input(double acceleration, double steering_rate) {
    acceleration_ = acceleration;
    steering_rate_ = steering_rate;
    input_vector_ << acceleration, steering_rate;
}

void SystemInput::update_input(const Eigen::Ref<const Eigen::Vector2d>& system_input) {
    acceleration_ = system_input[0];
    steering_rate_ = system_input[1];
    input_vector_ = system_input;
}

}  // namespace optimal_parking
