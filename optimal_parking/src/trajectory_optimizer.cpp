#include "optimal_parking/trajectory_optimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
namespace optimal_parking {
TrajectoryOptimizer::TrajectoryOptimizer(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);

    trajectory_time_ = config["trajectory_time"].as<double>();
    Ts_ = config["Ts"].as<double>();

    state_lowerbound_ = Eigen::Map<Eigen::Vector<double, 5>>(config["state_lowerbound"].as<std::vector<double>>().data());
    state_upperbound_ = Eigen::Map<Eigen::Vector<double, 5>>(config["state_upperbound"].as<std::vector<double>>().data());
    input_lowerbound_ = Eigen::Map<Eigen::Vector<double, 2>>(config["input_lowerbound"].as<std::vector<double>>().data());
    input_upperbound_ = Eigen::Map<Eigen::Vector<double, 2>>(config["input_upperbound"].as<std::vector<double>>().data());

    n_sqp_ = config["n_sqp"].as<int>();
    max_iteration_ = config["max_iteration"].as<int>();
    rho_slack_ = config["rho_slack"].as<double>();

    Eigen::Vector<double, 5> state_weight =
        Eigen::Map<Eigen::Vector<double, 5>>(config["state_weight"].as<std::vector<double>>().data());
    Eigen::Vector<double, 2> input_weight =
        Eigen::Map<Eigen::Vector<double, 2>>(config["input_weight"].as<std::vector<double>>().data());
    Q_ = state_weight.asDiagonal();
    R_ = input_weight.asDiagonal();

    prediction_horizon_ = static_cast<int>(std::ceil(trajectory_time_ / Ts_));
    state_dim_ = 5;
    input_dim_ = 2;

    nx_ = state_dim_ * (prediction_horizon_ + 1);
    nu_ = input_dim_ * prediction_horizon_;

    total_vars_ = nx_ + nu_;

    n_eq_ = state_dim_ * (prediction_horizon_ + 1) + state_dim_;
    n_ineq_ = total_vars_;

    n_slack_ = state_dim_;
    total_vars_slack_ = total_vars_ + n_slack_;

    x_goal_.setZero();
    x0_.setZero();
    u_goal_.setZero();
    initial_guess_.setZero(total_vars_);
    optimal_solution_.setZero(total_vars_);
}

TrajectoryOptimizer::TrajectoryOptimizer(const double& trajectory_time,
                                         const double& Ts,
                                         const int& n_sqp,
                                         const int& max_iteration,
                                         const double& rho_slack,
                                         const Eigen::Vector<double, 5>& state_weight,
                                         const Eigen::Vector<double, 2>& input_weight,
                                         const Eigen::Vector<double, 5>& state_lowerbound,
                                         const Eigen::Vector<double, 5>& state_upperbound,
                                         const Eigen::Vector<double, 2>& input_lowerbound,
                                         const Eigen::Vector<double, 2>& input_upperbound)
    : trajectory_time_(trajectory_time)
    , Ts_(Ts)
    , n_sqp_(n_sqp)
    , max_iteration_(max_iteration)
    , rho_slack_(rho_slack)
    , state_lowerbound_(state_lowerbound)
    , state_upperbound_(state_upperbound)
    , input_lowerbound_(input_lowerbound)
    , input_upperbound_(input_upperbound) {
    prediction_horizon_ = static_cast<int>(std::ceil(trajectory_time_ / Ts_));
    state_dim_ = 5;
    input_dim_ = 2;

    nx_ = state_dim_ * (prediction_horizon_ + 1);
    nu_ = input_dim_ * prediction_horizon_;

    total_vars_ = nx_ + nu_;

    n_eq_ = state_dim_ * (prediction_horizon_ + 1) + state_dim_;
    n_ineq_ = total_vars_;

    n_slack_ = state_dim_;
    total_vars_slack_ = total_vars_ + n_slack_;

    Q_ = state_weight.asDiagonal();
    R_ = input_weight.asDiagonal();

    x_goal_.setZero();
    x0_.setZero();
    u_goal_.setZero();
    initial_guess_.setZero(total_vars_);
    optimal_solution_.setZero(total_vars_);
}

void TrajectoryOptimizer::setGoalPose(const Eigen::Vector<double, 5>& goal_pose) { x_goal_ = goal_pose; }
void TrajectoryOptimizer::setInitialPose(const Eigen::Vector<double, 5>& initial_pose) { x0_ = initial_pose; }

void TrajectoryOptimizer::runSQP(const SystemModel& system_model) {
    initial_guess_.setZero(total_vars_);
    optimal_solution_.resize(total_vars_);
    for (int i = 0; i <= prediction_horizon_; ++i) {
        double alpha = static_cast<double>(i) / prediction_horizon_;
        initial_guess_.segment(i * state_dim_, state_dim_) = (1 - alpha) * x0_ + alpha * x_goal_;
    }

    optimal_solution_ = initial_guess_;

    for (int iter = 0; iter < n_sqp_; ++iter) {
        std::cout << "SQP Iteration: " << iter << std::endl;

        auto [hessian, gradient, linearMatrix, lowerBound, upperBound] = setupQP(system_model, Q_, R_);

        Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
        Eigen::SparseMatrix<double> linear_sparse = linearMatrix.sparseView();

        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.settings()->setMaxIteration(max_iteration_);
        solver.settings()->setAbsoluteTolerance(1e-3);
        solver.settings()->setRelativeTolerance(1e-3);
        solver.data()->setNumberOfVariables(total_vars_slack_);
        solver.data()->setNumberOfConstraints(n_eq_ + n_ineq_);
        solver.data()->setHessianMatrix(hessian_sparse);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(linear_sparse);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);
        solver.initSolver();

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            std::cerr << "QP failed at iteration " << iter << std::endl;
            return;
        }

        Eigen::VectorXd delta_solution = solver.getSolution();

        // Todo: Backtracking line search
        optimal_solution_ = optimal_solution_ + 0.1 * delta_solution.head(total_vars_);

        for (int i = 0; i < nx_; ++i) {
            optimal_solution_(i) =
                std::max(state_lowerbound_(i % state_dim_), std::min(state_upperbound_(i % state_dim_), optimal_solution_(i)));
        }
        for (int i = 0; i < nu_; ++i) {
            optimal_solution_(nx_ + i) = std::max(input_lowerbound_(i % input_dim_),
                                                  std::min(input_upperbound_(i % input_dim_), optimal_solution_(nx_ + i)));
        }

        std::cout << "Final goal pose: " << optimal_solution_.segment(nx_ - state_dim_, state_dim_).transpose() << std::endl;
    }

    updateTrajectoryData();
}

void TrajectoryOptimizer::updateTrajectoryData() {
    path_x_.clear();
    path_y_.clear();
    path_yaw_.clear();
    acceleration_.clear();
    steering_rate_.clear();

    for (int i = 0; i <= prediction_horizon_; ++i) {
        path_x_.push_back(optimal_solution_(state_dim_ * i));
        path_y_.push_back(optimal_solution_(state_dim_ * i + 1));
        path_yaw_.push_back(optimal_solution_(state_dim_ * i + 2));

        if (i < prediction_horizon_) {
            acceleration_.push_back(optimal_solution_(nx_ + input_dim_ * i));
            steering_rate_.push_back(optimal_solution_(nx_ + input_dim_ * i + 1));
        }
    }
}

QPData TrajectoryOptimizer::setupQP(const SystemModel& system_model,
                                    Eigen::Matrix<double, 5, 5>& Q,
                                    Eigen::Matrix<double, 2, 2>& R) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_vars_slack_, total_vars_slack_);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(total_vars_slack_);

    Eigen::MatrixXd Ceq = Eigen::MatrixXd::Zero(n_eq_, total_vars_slack_);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq_);

    Eigen::MatrixXd Cineq = Eigen::MatrixXd::Zero(n_ineq_, total_vars_slack_);
    Eigen::VectorXd bineq_lower = Eigen::VectorXd::Zero(n_ineq_);
    Eigen::VectorXd bineq_upper = Eigen::VectorXd::Zero(n_ineq_);

    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        SystemState xk_next(optimal_solution_.segment(state_dim_ * (time_step + 1), state_dim_));
        SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));

        auto [Ak, Bk, gk] = system_model.getSystemJacobian(xk, uk, Ts_);

        H.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) = Q;
        H.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) = R;

        Ceq.block(state_dim_ * (time_step + 1), state_dim_ * (time_step + 1), state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);

        Ceq.block(state_dim_ * (time_step + 1), state_dim_ * time_step, state_dim_, state_dim_) = -Ak;
        Ceq.block(state_dim_ * (time_step + 1), nx_ + input_dim_ * time_step, state_dim_, input_dim_) = -Bk;
        beq.segment(state_dim_ * (time_step + 1), state_dim_) = (Ak * xk() + Bk * uk() + gk) - xk_next();

        Cineq.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        bineq_lower.segment(state_dim_ * time_step, state_dim_) = state_lowerbound_ - xk();
        bineq_upper.segment(state_dim_ * time_step, state_dim_) = state_upperbound_ - xk();
        Cineq.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) =
            Eigen::MatrixXd::Identity(input_dim_, input_dim_);
        bineq_lower.segment(nx_ + input_dim_ * time_step, input_dim_) = input_lowerbound_ - uk();
        bineq_upper.segment(nx_ + input_dim_ * time_step, input_dim_) = input_upperbound_ - uk();
    }

    bineq_lower.segment(nx_ - state_dim_, state_dim_) =
        state_lowerbound_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);
    bineq_upper.segment(nx_ - state_dim_, state_dim_) =
        state_upperbound_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    Ceq.block(0, 0, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Ceq.block(n_eq_ - state_dim_, nx_ - state_dim_, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

    Ceq.block(n_eq_ - state_dim_, total_vars_, state_dim_, n_slack_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    beq.segment(n_eq_ - state_dim_, state_dim_) = x_goal_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    H.block(total_vars_, total_vars_, n_slack_, n_slack_) = rho_slack_ * Eigen::MatrixXd::Identity(n_slack_, n_slack_);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq_ + n_ineq_, total_vars_slack_);
    A.block(0, 0, n_eq_, total_vars_slack_) = Ceq;
    A.block(n_eq_, 0, n_ineq_, total_vars_slack_) = Cineq;

    Eigen::VectorXd lowerBound(n_eq_ + n_ineq_);
    Eigen::VectorXd upperBound(n_eq_ + n_ineq_);

    lowerBound.head(n_eq_) = beq;
    upperBound.head(n_eq_) = beq;
    lowerBound.tail(n_ineq_) = bineq_lower;
    upperBound.tail(n_ineq_) = bineq_upper;

    return {H, f, A, lowerBound, upperBound};
}

}  // namespace optimal_parking
