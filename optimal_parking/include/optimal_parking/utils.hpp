#ifndef UTILS_HPP
#define UTILS_HPP

#include <array>
#include <limits>
#include <utility>

#include "optimal_parking/system/system_model.hpp"
namespace optimal_parking {
class Utils {
public:
    inline static std::pair<double, double> find_closest_point_on_obstacle(const double& x,
                                                                           const double& y,
                                                                           const Obstacle& obstacle) {
        Eigen::Vector2d vehicle_pos(x, y);

        Eigen::Vector2d obs_center = obstacle.center;
        double half_length = obstacle.length / 2.0;
        double half_width = obstacle.width / 2.0;
        double yaw = obstacle.yaw;

        Eigen::Rotation2Dd rotation(yaw);

        std::array<Eigen::Vector2d, 4> corners;
        corners[0] = Eigen::Vector2d(-half_length, -half_width);
        corners[1] = Eigen::Vector2d(half_length, -half_width);
        corners[2] = Eigen::Vector2d(half_length, half_width);
        corners[3] = Eigen::Vector2d(-half_length, half_width);

        for (Eigen::Vector2d& corner : corners) {
            corner = rotation * corner + obs_center;
        }

        double min_dist = std::numeric_limits<double>::infinity();
        Eigen::Vector2d closest_point;

        for (int i = 0; i < 4; ++i) {
            int next_i = (i + 1) % 4;
            Eigen::Vector2d p1 = corners[i];
            Eigen::Vector2d p2 = corners[next_i];

            Eigen::Vector2d closest_on_edge = get_closest_point_on_segment(p1, p2, vehicle_pos);

            double dist = (vehicle_pos - closest_on_edge).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_point = closest_on_edge;
            }
        }

        return std::make_pair(closest_point.x(), closest_point.y());
    }

    inline static Eigen::Vector2d get_closest_point_on_segment(const Eigen::Vector2d& p1,
                                                               const Eigen::Vector2d& p2,
                                                               const Eigen::Vector2d& point) {
        Eigen::Vector2d line_vec = p2 - p1;
        Eigen::Vector2d point_vec = point - p1;

        const double line_length_squared = line_vec.squaredNorm();
        if (line_length_squared <= std::numeric_limits<double>::epsilon()) {
            return p1;
        }
        double t = point_vec.dot(line_vec) / line_length_squared;

        t = std::max(0.0, std::min(1.0, t));
        return p1 + t * line_vec;
    }

    inline static Eigen::Vector<double, 5> rk4(const SystemModel& model,
                                               const Eigen::Vector<double, 5>& state,
                                               const Eigen::Vector<double, 2>& input,
                                               const double& time_step) {
        Eigen::Vector<double, 5> k1 = model.evaluate_dynamics(SystemState(state), SystemInput(input));
        Eigen::Vector<double, 5> k2 = model.evaluate_dynamics(SystemState(state + time_step / 2 * k1), SystemInput(input));
        Eigen::Vector<double, 5> k3 = model.evaluate_dynamics(SystemState(state + time_step / 2 * k2), SystemInput(input));
        Eigen::Vector<double, 5> k4 = model.evaluate_dynamics(SystemState(state + time_step * k3), SystemInput(input));

        return state + time_step * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6);
    }

    inline static Eigen::Vector<double, 5> euler_forward(const SystemModel& model,
                                                         const Eigen::Vector<double, 5>& state,
                                                         const Eigen::Vector<double, 2>& input,
                                                         const double& time_step) {
        Eigen::Vector<double, 5> state_dot = model.evaluate_dynamics(SystemState(state), SystemInput(input));
        return state + time_step * state_dot;
    }
};
}  // namespace optimal_parking
#endif  // MPCC_UTILS_HPP
