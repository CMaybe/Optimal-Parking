#ifndef SYSTEM_MODEL_HPP
#define SYSTEM_MODEL_HPP

#include <Eigen/Dense>
#include <cmath>

#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_state.hpp"

namespace optimal_parking {

struct ModelMatrices {
    Eigen::Matrix<double, 5, 5> A;
    Eigen::Matrix<double, 5, 2> B;
    Eigen::Matrix<double, 5, 1> g;
};

class SystemModel {
public:
    SystemModel() = default;
    SystemModel(const double& dt, const double& wheel_radius, const double& car_length, const double& car_width);
    SystemModel(const std::string& path);
    SystemModel(const SystemModel& other);
    void initialize(const std::string& path);
    void initialize(const double& dt, const double& wheel_radius, const double& car_length, const double& car_width);

    Eigen::Vector<double, 5> f(const SystemState& state, const SystemInput& input) const;
    ModelMatrices getSystemJacobian(const SystemState& state, const SystemInput& input);
    inline Eigen::Matrix<double, 5, 5> Ac() const { return Ac_; };
    inline Eigen::Matrix<double, 5, 5> Ad() const { return Ad_; };
    inline Eigen::Matrix<double, 5, 2> Bc() const { return Bc_; };
    inline Eigen::Matrix<double, 5, 2> Bd() const { return Bd_; };
    inline double dt() const { return dt_; };
    inline double wheel_radius() const { return wheel_radius_; };
    inline double car_length() const { return car_length_; };
    inline double car_width() const { return car_width_; };

private:
    double dt_;
    double wheel_radius_;
    double car_length_;
    double car_width_;

    Eigen::Matrix<double, 5, 5> Ac_;
    Eigen::Matrix<double, 5, 2> Bc_;
    Eigen::Matrix<double, 5, 1> gc_;

    Eigen::Matrix<double, 5, 5> Ad_;
    Eigen::Matrix<double, 5, 2> Bd_;
    Eigen::Matrix<double, 5, 1> gd_;
};

}  // namespace optimal_parking

#endif  // SYSTEM_MODEL_HPP