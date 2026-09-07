#pragma once

#include <Eigen/Dense>

namespace optimal_parking {
class SystemState {
public:
    SystemState();
    SystemState(double x, double y, double yaw, double velocity, double delta);
    SystemState(const SystemState& other);
    SystemState(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state);
    SystemState& operator=(const SystemState& other);
    SystemState& operator=(const Eigen::Vector<double, 5>& system_state);

    void update_state(double x, double y, double yaw, double velocity, double delta);
    void update_state(const Eigen::Ref<const Eigen::Vector<double, 5>>& system_state);

    [[nodiscard]] inline double x() const { return x_; };
    [[nodiscard]] inline double y() const { return y_; };
    [[nodiscard]] inline double yaw() const { return yaw_; };
    [[nodiscard]] inline double velocity() const { return velocity_; }
    [[nodiscard]] inline double delta() const { return delta_; }

    inline const Eigen::Vector<double, 5>& operator()() const { return system_state_; };
    inline double operator()(const int& idx) const { return system_state_[idx]; };
    inline double operator[](const int& idx) const { return system_state_[idx]; };

    friend Eigen::Vector<double, 5> operator*(const Eigen::Matrix<double, 5, 5>& lhs, const SystemState& rhs) {
        return lhs * rhs.system_state_;
    }

private:
    Eigen::Vector<double, 5> system_state_;
    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double delta_;
};

}  // namespace optimal_parking
