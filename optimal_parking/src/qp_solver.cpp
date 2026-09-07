#include "optimal_parking/qp_solver.hpp"

#include <OsqpEigen/OsqpEigen.h>

namespace optimal_parking {

std::optional<Eigen::VectorXd> solve_qp(const QPData& problem, const QPSolverSettings& settings) {
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(settings.warm_start);
    solver.settings()->setVerbosity(settings.verbose);
    solver.settings()->setMaxIteration(settings.max_iterations);
    solver.settings()->setAbsoluteTolerance(settings.absolute_tolerance);
    solver.settings()->setRelativeTolerance(settings.relative_tolerance);

    const Eigen::SparseMatrix<double> hessian = problem.H.sparseView();
    const Eigen::SparseMatrix<double> constraints = problem.A.sparseView();
    Eigen::VectorXd gradient = problem.f;
    Eigen::VectorXd lower_bound = problem.lower_bound;
    Eigen::VectorXd upper_bound = problem.upper_bound;
    solver.data()->setNumberOfVariables(static_cast<int>(problem.H.rows()));
    solver.data()->setNumberOfConstraints(static_cast<int>(problem.A.rows()));
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(constraints);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    if (!solver.initSolver() || solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        return std::nullopt;
    }
    return solver.getSolution();
}

}  // namespace optimal_parking
