#ifndef SYSTEM_MODEL_HPP
#define SYSTEM_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>

#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_state.hpp"

namespace optimal_parking {

struct ModelMatrices {
    Eigen::Matrix<double, 5, 5> Ad;
    Eigen::Matrix<double, 5, 2> Bd;
    Eigen::Matrix<double, 5, 1> gd;
};

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(const double& car_length, const double& car_width);
    SystemModel(const std::string& path);
    SystemModel(const SystemModel& other);
    void initialize(const std::string& path);
    void initialize(const double& car_length, const double& car_width);

    Eigen::Vector<double, 5> f(const SystemState& state, const SystemInput& input) const;
    ModelMatrices getSystemJacobian(const SystemState& state, const SystemInput& input, const double& dt) const;
    inline double car_length() const { return car_length_; };
    inline double car_width() const { return car_width_; };

private:
    double car_length_;
    double car_width_;
};

}  // namespace optimal_parking

#endif  // SYSTEM_MODEL_HPP