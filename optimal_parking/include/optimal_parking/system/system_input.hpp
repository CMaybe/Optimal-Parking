#ifndef SYSTEM_INPUT_HPP
#define SYSTEM_INPUT_HPP

#include <Eigen/Dense>

namespace optimal_parking {
class SystemInput {
public:
    SystemInput();
    SystemInput(const double& acceleration, const double& steering_rate);
    SystemInput(const SystemInput& other);
    SystemInput(const Eigen::Ref<const Eigen::Vector2d>& system_input);
    SystemInput& operator=(const SystemInput& other);

    void updateInput(const double& acceleration, const double& steering_rate);
    void updateInput(const Eigen::Ref<const Eigen::Vector2d>& system_input);

    inline double acceleration() const { return acceleration_; };
    inline double steering_rate() const { return steering_rate_; };

    inline const Eigen::Vector2d& operator()() const { return system_input_; };
    inline double operator()(const int& idx) const { return system_input_[idx]; };
    inline double operator[](const int& idx) const { return system_input_[idx]; };

    friend Eigen::Vector<double, 5> operator*(const Eigen::Matrix<double, 5, 2>& lhs, const SystemInput& rhs) {
        return lhs * rhs.system_input_;
    }

private:
    double acceleration_;
    double steering_rate_;

    Eigen::Vector2d system_input_;
};

}  // namespace optimal_parking

#endif  // SYSTEM_INPUT_HPP