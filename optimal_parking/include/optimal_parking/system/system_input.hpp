#pragma once

#include <Eigen/Dense>

namespace optimal_parking {
class SystemInput {
public:
    SystemInput();
    SystemInput(double acceleration, double steering_rate);
    SystemInput(const SystemInput& other);
    SystemInput(const Eigen::Ref<const Eigen::Vector2d>& system_input);
    SystemInput& operator=(const SystemInput& other);

    void update_input(double acceleration, double steering_rate);
    void update_input(const Eigen::Ref<const Eigen::Vector2d>& system_input);

    [[nodiscard]] inline double acceleration() const { return acceleration_; };
    [[nodiscard]] inline double steering_rate() const { return steering_rate_; };

    inline const Eigen::Vector2d& operator()() const { return input_vector_; };
    inline double operator()(const int& idx) const { return input_vector_[idx]; };
    inline double operator[](const int& idx) const { return input_vector_[idx]; };

    friend Eigen::Vector<double, 5> operator*(const Eigen::Matrix<double, 5, 2>& lhs, const SystemInput& rhs) {
        return lhs * rhs.input_vector_;
    }

private:
    double acceleration_;
    double steering_rate_;

    Eigen::Vector2d input_vector_;
};

}  // namespace optimal_parking
