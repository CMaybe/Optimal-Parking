#ifndef UTILS_HPP
#define UTILS_HPP

#include "optimal_parking/system/system_model.hpp"
namespace optimal_parking {
class utils {
public:
    inline static Eigen::Vector<double, 5> RK4(const SystemModel &model,
                                               const Eigen::Vector<double, 5> &x,
                                               const Eigen::Vector<double, 2> &u,
                                               const double &ts) {
        Eigen::Vector<double, 5> k1 = model.f(SystemState(x), SystemInput(u));
        Eigen::Vector<double, 5> k2 = model.f(SystemState(x + ts / 2 * k1), SystemInput(u));
        Eigen::Vector<double, 5> k3 = model.f(SystemState(x + ts / 2 * k2), SystemInput(u));
        Eigen::Vector<double, 5> k4 = model.f(SystemState(x + ts * k3), SystemInput(u));

        return x + ts * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6);
    }

    inline static Eigen::Vector<double, 5> EF(const SystemModel &model,
                                              const Eigen::Vector<double, 5> &x,
                                              const Eigen::Vector<double, 2> &u,
                                              const double &ts) {
        Eigen::Vector<double, 5> f = model.f(SystemState(x), SystemInput(u));
        return x + ts * f;
    }
};
}  // namespace optimal_parking
#endif  // MPCC_UTILS_HPP
