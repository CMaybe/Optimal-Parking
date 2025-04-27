#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <Eigen/Dense>

namespace optimal_parking {
class SystemState {
public:
    SystemState() = default;
    SystemState(const double& x, const double& y, const double& yaw, const double& velocity, const double& delta);
    SystemState(const SystemState& other);
    SystemState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state);
    SystemState& operator=(const SystemState& other);

    void updateState(const double& x, const double& y, const double& yaw, const double& velocity, const double& delta);
    void updateState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state);

    inline double x() const { return x_; };
    inline double y() const { return y_; };
    inline double yaw() const { return yaw_; };
    inline double velocity() const { return velocity_; }
    inline double delta() const { return delta_; }

    inline const Eigen::Vector<double, 5>& operator()() const { return system_state_; };
    inline double operator()(const int& idx) const { return system_state_[idx]; };
    inline double operator[](const int& idx) const { return system_state_[idx]; };

    friend Eigen::Vector<double, 5> operator*(const Eigen::Matrix<double, 5, 5>& lhs, const SystemState& rhs) {
        return lhs * rhs.system_state_;
    }

private:
    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double delta_;

    Eigen::Vector<double, 5> system_state_;
};

}  // namespace optimal_parking

#endif  // SYSTEM_STATE_HPP