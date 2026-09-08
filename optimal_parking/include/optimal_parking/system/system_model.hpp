#pragma once

#include <Eigen/Dense>
#include <string>

#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_state.hpp"
#include "optimal_parking/types.hpp"

namespace optimal_parking {

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(double vehicle_length, double vehicle_width);
    SystemModel(const std::string& path);
    void initialize(const std::string& path);
    void initialize(double vehicle_length, double vehicle_width);

    [[nodiscard]] Eigen::Vector<double, 5> evaluate_dynamics(const SystemState& state, const SystemInput& input) const;
    [[nodiscard]] ModelMatrices compute_discrete_linearization(const SystemState& state,
                                                               const SystemInput& input,
                                                               double time_step) const;
    [[nodiscard]] inline double vehicle_length() const { return vehicle_length_; };
    [[nodiscard]] inline double vehicle_width() const { return vehicle_width_; };

private:
    double vehicle_length_ = 0.0;
    double vehicle_width_ = 0.0;
};

}  // namespace optimal_parking
