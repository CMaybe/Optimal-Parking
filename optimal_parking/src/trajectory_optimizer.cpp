#include "optimal_parking/trajectory_optimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "optimal_parking/rrt_star.hpp"
#include "optimal_parking/utils.hpp"

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
    rho_goal_ = config["rho_goal"].as<double>();
    rho_obs_ = config["rho_obs"].as<double>();
    safety_margin_ = config["safety_margin"].as<double>();

    for (const auto& obs : config["obstacles"]) {
        Obstacle obstacle;
        obstacle.center = Eigen::Map<Eigen::Vector2d>(obs["center"].as<std::vector<double>>().data());
        obstacle.length = obs["length"].as<double>();
        obstacle.width = obs["width"].as<double>();
        obstacle.yaw = obs["yaw"].as<double>();
        obstacles_.push_back(obstacle);
    }

    rrt_star_ = std::make_unique<RRTStar>(obstacles_,
                                          config["map_x_min"].as<double>(),
                                          config["map_x_max"].as<double>(),
                                          config["map_y_min"].as<double>(),
                                          config["map_y_max"].as<double>(),
                                          config["goal_radius"].as<double>(),
                                          config["goal_bias"].as<double>(),
                                          config["step_dist"].as<double>(),
                                          config["rewire_radius"].as<double>(),
                                          config["max_iterations"].as<int>(),
                                          config["vehicle_length"].as<double>(),
                                          config["vehicle_width"].as<double>());

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

    n_obstacle_constraints_ = (prediction_horizon_ + 1) * obstacles_.size();
    n_obstacle_slack_ = n_obstacle_constraints_;
    total_vars_all_slack_ = total_vars_ + n_slack_ + n_obstacle_slack_;
    total_constraints_ = n_eq_ + n_ineq_ + n_obstacle_constraints_ + n_obstacle_slack_;

    x_goal_.setZero();
    x0_.setZero();
    u_goal_.setZero();
    initial_guess_.setZero(total_vars_);
    optimal_solution_.setZero(total_vars_);
}

void TrajectoryOptimizer::setObstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
    n_obstacle_constraints_ = (prediction_horizon_ + 1) * obstacles_.size();
    n_obstacle_slack_ = n_obstacle_constraints_;
    total_vars_all_slack_ = total_vars_ + n_slack_ + n_obstacle_slack_;
    total_constraints_ = n_eq_ + n_ineq_ + n_obstacle_constraints_ + n_obstacle_slack_;
}

void TrajectoryOptimizer::setGoalPose(const Eigen::Vector<double, 5>& goal_pose) { x_goal_ = goal_pose; }
void TrajectoryOptimizer::setInitialPose(const Eigen::Vector<double, 5>& initial_pose) { x0_ = initial_pose; }

void TrajectoryOptimizer::runSQP(const SystemModel& system_model) {
    initial_guess_.setZero(total_vars_);
    optimal_solution_.resize(total_vars_);
    vehicle_radius_ = std::sqrt(std::pow(system_model.vehicle_length(), 2) + std::pow(system_model.vehicle_width(), 2));

    std::vector<std::shared_ptr<Node>> nodes;
    Eigen::Vector3d start(x0_(0), x0_(1), x0_(2));
    Eigen::Vector3d goal(x_goal_(0), x_goal_(1), x_goal_(2));
    std::vector<Eigen::Vector3d> path = rrt_star_->makePath(start, goal, prediction_horizon_ + 1);
    if (path.empty() == false) {
        for (int i = 0; i <= prediction_horizon_; ++i) {
            initial_guess_(state_dim_ * i) = path[i](0);
            initial_guess_(state_dim_ * i + 1) = path[i](1);
            initial_guess_(state_dim_ * i + 2) = path[i](2);
        }
    } else {
        std::cerr << "RRT* path planning failed." << std::endl;
        return;
    }
    optimal_solution_ = initial_guess_;
    std::cout << "Final goal pose: " << optimal_solution_.segment(nx_ - state_dim_, state_dim_).transpose() << std::endl;

    for (int iter = 0; iter < n_sqp_; ++iter) {
        std::cout << "SQP Iteration: " << iter << std::endl;

        auto [hessian, gradient, linearMatrix, lowerBound, upperBound] = setupQP(system_model, Q_, R_);

        Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
        Eigen::SparseMatrix<double> linear_sparse = linearMatrix.sparseView();

        std::unique_ptr<OsqpEigen::Solver> solver = std::make_unique<OsqpEigen::Solver>();

        solver->settings()->setWarmStart(true);
        solver->settings()->setVerbosity(false);
        solver->settings()->setMaxIteration(max_iteration_);
        solver->settings()->setAbsoluteTolerance(1e-3);
        solver->settings()->setRelativeTolerance(1e-3);
        solver->data()->setNumberOfVariables(total_vars_all_slack_);
        solver->data()->setNumberOfConstraints(total_constraints_);
        solver->data()->setHessianMatrix(hessian_sparse);
        solver->data()->setGradient(gradient);
        solver->data()->setLinearConstraintsMatrix(linear_sparse);
        solver->data()->setLowerBound(lowerBound);
        solver->data()->setUpperBound(upperBound);
        solver->initSolver();

        if (solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            std::cerr << "QP failed at iteration " << iter << std::endl;
            return;
        }

        Eigen::VectorXd delta_solution = solver->getSolution();

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
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_vars_all_slack_, total_vars_all_slack_);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(total_vars_all_slack_);

    Eigen::MatrixXd Ceq = Eigen::MatrixXd::Zero(n_eq_, total_vars_all_slack_);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq_);

    Eigen::MatrixXd Cineq = Eigen::MatrixXd::Zero(n_ineq_, total_vars_all_slack_);
    Eigen::VectorXd bineq_lower = Eigen::VectorXd::Zero(n_ineq_);
    Eigen::VectorXd bineq_upper = Eigen::VectorXd::Zero(n_ineq_);

    // Hessian setup (cost function)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        H.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) = Q;
        H.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) = R;
    }
    H.block(nx_ - state_dim_, nx_ - state_dim_, state_dim_, state_dim_) = Q;

    // Slack penalties
    H.block(total_vars_, total_vars_, n_slack_, n_slack_) = rho_goal_ * Eigen::MatrixXd::Identity(n_slack_, n_slack_);
    H.block(total_vars_ + n_slack_, total_vars_ + n_slack_, n_obstacle_slack_, n_obstacle_slack_) =
        rho_obs_ * Eigen::MatrixXd::Identity(n_obstacle_slack_, n_obstacle_slack_);

    // Equality constraints (dynamics and initial/goal)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        SystemState xk_next(optimal_solution_.segment(state_dim_ * (time_step + 1), state_dim_));
        SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));

        auto [Ak, Bk, gk] = system_model.getSystemJacobian(xk, uk, Ts_);

        Ceq.block(state_dim_ * (time_step + 1), state_dim_ * (time_step + 1), state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        Ceq.block(state_dim_ * (time_step + 1), state_dim_ * time_step, state_dim_, state_dim_) = -Ak;
        Ceq.block(state_dim_ * (time_step + 1), nx_ + input_dim_ * time_step, state_dim_, input_dim_) = -Bk;
        beq.segment(state_dim_ * (time_step + 1), state_dim_) = (Ak * xk() + Bk * uk() + gk) - xk_next();
    }
    Ceq.block(0, 0, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Ceq.block(n_eq_ - state_dim_, nx_ - state_dim_, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Ceq.block(n_eq_ - state_dim_, total_vars_, state_dim_, n_slack_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    beq.segment(n_eq_ - state_dim_, state_dim_) = x_goal_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    // Inequality constraints (state/input bounds)
    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        Cineq.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        bineq_lower.segment(state_dim_ * time_step, state_dim_) = state_lowerbound_ - xk();
        bineq_upper.segment(state_dim_ * time_step, state_dim_) = state_upperbound_ - xk();
        if (time_step < prediction_horizon_) {
            SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));
            Cineq.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) =
                Eigen::MatrixXd::Identity(input_dim_, input_dim_);
            bineq_lower.segment(nx_ + input_dim_ * time_step, input_dim_) = input_lowerbound_ - uk();
            bineq_upper.segment(nx_ + input_dim_ * time_step, input_dim_) = input_upperbound_ - uk();
        }
    }

    Eigen::MatrixXd Cobs = Eigen::MatrixXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_, total_vars_all_slack_);
    Eigen::VectorXd bobs_lower = Eigen::VectorXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_);
    Eigen::VectorXd bobs_upper = Eigen::VectorXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_);

    int constraint_idx = 0;
    int slack_idx = total_vars_ + n_slack_;

    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        double x_k = optimal_solution_(state_dim_ * time_step);
        double y_k = optimal_solution_(state_dim_ * time_step + 1);

        for (const Obstacle& obs : obstacles_) {
            auto [c_x, c_y] = utils::findClosestPointOnObstacle(x_k, y_k, obs);
            double d_safe = vehicle_radius_ + safety_margin_;

            double dx = x_k - c_x;
            double dy = y_k - c_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            double grad_x = dist > 1e-4 ? dx / dist : 0.0;
            double grad_y = dist > 1e-4 ? dy / dist : 0.0;

            Cobs(constraint_idx, state_dim_ * time_step) = grad_x;
            Cobs(constraint_idx, state_dim_ * time_step + 1) = grad_y;
            Cobs(constraint_idx, slack_idx) = 1.0;
            Cobs(n_obstacle_constraints_ + constraint_idx, slack_idx) = 1.0;
            bobs_lower(constraint_idx) = d_safe - dist;
            bobs_upper(constraint_idx) = std::numeric_limits<double>::infinity();
            bobs_lower(n_obstacle_constraints_ + constraint_idx) = 0.0;
            bobs_upper(n_obstacle_constraints_ + constraint_idx) = std::numeric_limits<double>::infinity();

            constraint_idx++;
            slack_idx++;
        }
    }

    // Combine all constraints
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(total_constraints_, total_vars_all_slack_);
    Eigen::VectorXd lowerBound(total_constraints_);
    Eigen::VectorXd upperBound(total_constraints_);

    A.block(0, 0, n_eq_, total_vars_all_slack_) = Ceq;
    A.block(n_eq_, 0, n_ineq_, total_vars_all_slack_) = Cineq;
    A.block(n_eq_ + n_ineq_, 0, n_obstacle_constraints_ + n_obstacle_slack_, total_vars_all_slack_) = Cobs;

    lowerBound.head(n_eq_) = beq;
    upperBound.head(n_eq_) = beq;
    lowerBound.segment(n_eq_, n_ineq_) = bineq_lower;
    upperBound.segment(n_eq_, n_ineq_) = bineq_upper;
    lowerBound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_lower;
    upperBound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_upper;

    return {H, f, A, lowerBound, upperBound};
}

}  // namespace optimal_parking