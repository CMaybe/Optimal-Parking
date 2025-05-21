#ifndef UTILS_HPP
#define UTILS_HPP

#include "optimal_parking/system/system_model.hpp"
namespace optimal_parking {
class utils {
public:
    inline static std::pair<double, double> findClosestPointOnObstacle(const double &xk, const double &yk, const Obstacle &obs) {
        Eigen::Vector2d vehicle_pos(xk, yk);

        Eigen::Vector2d obs_center = obs.center;
        double half_length = obs.length / 2.0;
        double half_width = obs.width / 2.0;
        double yaw = obs.yaw;

        Eigen::Rotation2Dd rotation(yaw);

        std::vector<Eigen::Vector2d> corners(4);
        corners[0] = Eigen::Vector2d(-half_length, -half_width);
        corners[1] = Eigen::Vector2d(half_length, -half_width);
        corners[2] = Eigen::Vector2d(half_length, half_width);
        corners[3] = Eigen::Vector2d(-half_length, half_width);

        for (Eigen::Vector2d &corner : corners) {
            corner = rotation * corner + obs_center;
        }

        double min_dist = std::numeric_limits<double>::infinity();
        Eigen::Vector2d closest_point;

        for (int i = 0; i < 4; ++i) {
            int next_i = (i + 1) % 4;
            Eigen::Vector2d p1 = corners[i];
            Eigen::Vector2d p2 = corners[next_i];

            Eigen::Vector2d closest_on_edge = getClosestPointOnSegment(p1, p2, vehicle_pos);

            double dist = (vehicle_pos - closest_on_edge).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_point = closest_on_edge;
            }
        }

        return std::make_pair(closest_point.x(), closest_point.y());
    }

    inline static Eigen::Vector2d getClosestPointOnSegment(const Eigen::Vector2d &p1,
                                                           const Eigen::Vector2d &p2,
                                                           const Eigen::Vector2d &point) {
        Eigen::Vector2d line_vec = p2 - p1;
        Eigen::Vector2d point_vec = point - p1;

        double t = point_vec.dot(line_vec) / line_vec.squaredNorm();

        t = std::max(0.0, std::min(1.0, t));
        return p1 + t * line_vec;
    }

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
