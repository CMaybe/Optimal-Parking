#ifndef SYSTEM_MODEL_HPP
#define SYSTEM_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>

#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_state.hpp"
#include "optimal_parking/types.hpp"

namespace optimal_parking {

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(const double& vehicle_length, const double& vehicle_width);
    SystemModel(const std::string& path);
    SystemModel(const SystemModel& other);
    void initialize(const std::string& path);
    void initialize(const double& vehicle_length, const double& vehicle_width);

    Eigen::Vector<double, 5> f(const SystemState& state, const SystemInput& input) const;
    ModelMatrices getSystemJacobian(const SystemState& state, const SystemInput& input, const double& dt) const;
    inline double vehicle_length() const { return vehicle_length_; };
    inline double vehicle_width() const { return vehicle_width_; };

private:
    double vehicle_length_;
    double vehicle_width_;
};

}  // namespace optimal_parking

#endif  // SYSTEM_MODEL_HPP