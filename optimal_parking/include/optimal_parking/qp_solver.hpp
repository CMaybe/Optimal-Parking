#pragma once

#include <Eigen/Dense>
#include <optional>

#include "optimal_parking/types.hpp"

namespace optimal_parking {

struct QPSolverSettings {
    int max_iterations = 1000;
    double absolute_tolerance = 1e-3;
    double relative_tolerance = 1e-3;
    bool warm_start = true;
    bool verbose = false;
};

[[nodiscard]] std::optional<Eigen::VectorXd> solve_qp(const QPData& problem, const QPSolverSettings& settings);

}  // namespace optimal_parking
