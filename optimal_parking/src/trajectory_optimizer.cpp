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
    ts_ = config["Ts"].as<double>();

    state_lowerbound_ = Eigen::Map<Eigen::Vector<double, 5>>(config["state_lowerbound"].as<std::vector<double>>().data());
    state_upperbound_ = Eigen::Map<Eigen::Vector<double, 5>>(config["state_upperbound"].as<std::vector<double>>().data());
    input_lowerbound_ = Eigen::Map<Eigen::Vector<double, 2>>(config["input_lowerbound"].as<std::vector<double>>().data());
    input_upperbound_ = Eigen::Map<Eigen::Vector<double, 2>>(config["input_upperbound"].as<std::vector<double>>().data());

    n_sqp_ = config["n_sqp"].as<int>();
    qp_iteration_ = config["qp_iteration"].as<int>();
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
    q_ = state_weight.asDiagonal();
    r_ = input_weight.asDiagonal();

    prediction_horizon_ = static_cast<Eigen::Index>(std::ceil(trajectory_time_ / ts_));
    state_dim_ = 5;
    input_dim_ = 2;

    nx_ = state_dim_ * (prediction_horizon_ + 1);
    nu_ = input_dim_ * prediction_horizon_;
    total_vars_ = nx_ + nu_;

    n_eq_ = state_dim_ * (prediction_horizon_ + 1) + state_dim_;
    n_ineq_ = total_vars_;
    n_slack_ = state_dim_;
    total_vars_slack_ = total_vars_ + n_slack_;

    n_obstacle_constraints_ = (prediction_horizon_ + 1) * static_cast<Eigen::Index>(obstacles_.size());
    n_obstacle_slack_ = n_obstacle_constraints_;
    total_vars_all_slack_ = total_vars_ + n_slack_ + n_obstacle_slack_;
    total_constraints_ = n_eq_ + n_ineq_ + n_obstacle_constraints_ + n_obstacle_slack_;

    x_goal_.setZero();
    x0_.setZero();
    u_goal_.setZero();
    initial_guess_.setZero(total_vars_);
    optimal_solution_.setZero(total_vars_);
}

void TrajectoryOptimizer::set_obstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
    rrt_star_->set_obstacles(obstacles_);
    n_obstacle_constraints_ = (prediction_horizon_ + 1) * static_cast<Eigen::Index>(obstacles_.size());
    n_obstacle_slack_ = n_obstacle_constraints_;
    total_vars_all_slack_ = total_vars_ + n_slack_ + n_obstacle_slack_;
    total_constraints_ = n_eq_ + n_ineq_ + n_obstacle_constraints_ + n_obstacle_slack_;
}

void TrajectoryOptimizer::set_goal_pose(const Eigen::Vector<double, 5>& goal_pose) { x_goal_ = goal_pose; }
void TrajectoryOptimizer::set_initial_pose(const Eigen::Vector<double, 5>& initial_pose) { x0_ = initial_pose; }

void TrajectoryOptimizer::run_sqp(const SystemModel& system_model) {
    initial_guess_.setZero(total_vars_);
    optimal_solution_.resize(total_vars_);
    vehicle_radius_ = 0.5 * std::hypot(system_model.vehicle_length(), system_model.vehicle_width());

    std::vector<std::shared_ptr<Node>> nodes;
    Eigen::Vector3d start(x0_(0), x0_(1), x0_(2));
    Eigen::Vector3d goal(x_goal_(0), x_goal_(1), x_goal_(2));
    std::vector<Eigen::Vector3d> path = rrt_star_->make_path(start, goal, prediction_horizon_ + 1);
    if (path.size() == static_cast<std::size_t>(prediction_horizon_ + 1)) {
        initial_guess_.segment(0, state_dim_) = x0_;
        for (int i = 0; i <= prediction_horizon_; ++i) {
            initial_guess_(state_dim_ * i) = path[i](0);
            initial_guess_(state_dim_ * i + 1) = path[i](1);
            initial_guess_(state_dim_ * i + 2) = path[i](2);
        }
    } else {
        std::cerr << "RRT* path planning failed to produce the requested horizon." << std::endl;
        return;
    }
    optimal_solution_ = initial_guess_;

    for (int iter = 0; iter < n_sqp_; ++iter) {
        auto [hessian, gradient, linearMatrix, lowerBound, upperBound] = setup_qp(system_model, q_, r_);

        Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
        Eigen::SparseMatrix<double> linear_sparse = linearMatrix.sparseView();

        std::unique_ptr<OsqpEigen::Solver> solver = std::make_unique<OsqpEigen::Solver>();

        solver->settings()->setWarmStart(true);
        solver->settings()->setVerbosity(false);
        solver->settings()->setMaxIteration(qp_iteration_);
        solver->settings()->setAbsoluteTolerance(1e-3);
        solver->settings()->setRelativeTolerance(1e-3);
        solver->data()->setNumberOfVariables(static_cast<int>(total_vars_all_slack_));
        solver->data()->setNumberOfConstraints(static_cast<int>(total_constraints_));
        solver->data()->setHessianMatrix(hessian_sparse);
        solver->data()->setGradient(gradient);
        solver->data()->setLinearConstraintsMatrix(linear_sparse);
        solver->data()->setLowerBound(lowerBound);
        solver->data()->setUpperBound(upperBound);
        if (!solver->initSolver()) {
            std::cerr << "QP initialization failed at iteration " << iter << std::endl;
            return;
        }

        if (solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            std::cerr << "QP failed at iteration " << iter << std::endl;
            return;
        }

        Eigen::VectorXd delta_solution = solver->getSolution();
        const Eigen::VectorXd delta_variables = delta_solution.head(total_vars_);
        std::cout << "Iteration " << iter << ": delta_solution norm = " << delta_variables.norm() << std::endl;

        if (delta_variables.norm() < 0.05) {
            std::cout << "Delta solution norm: " << delta_variables.norm() << "\nConverged at iteration " << iter << "\n";
            break;
        }

        const auto merit = [](const Eigen::VectorXd& step,
                              const Eigen::MatrixXd& hessian_matrix,
                              const Eigen::VectorXd& gradient_vector,
                              const Eigen::MatrixXd& constraint_matrix,
                              const Eigen::VectorXd& lower_bound,
                              const Eigen::VectorXd& upper_bound) {
            const Eigen::VectorXd constraint_values = constraint_matrix * step;
            double violation = 0.0;
            for (Eigen::Index i = 0; i < constraint_values.size(); ++i) {
                violation += std::max(lower_bound(i) - constraint_values(i), 0.0);
                violation += std::max(constraint_values(i) - upper_bound(i), 0.0);
            }
            return 0.5 * step.dot(hessian_matrix * step) + gradient_vector.dot(step) + 1e4 * violation;
        };

        const double current_merit =
            merit(Eigen::VectorXd::Zero(total_vars_all_slack_), hessian, gradient, linearMatrix, lowerBound, upperBound);
        double step_length = 1.0;
        while (step_length > 1e-3 &&
               merit(step_length * delta_solution, hessian, gradient, linearMatrix, lowerBound, upperBound) > current_merit) {
            step_length *= 0.5;
        }
        optimal_solution_ += step_length * delta_variables;

        for (int i = 0; i < nx_; ++i) {
            optimal_solution_(i) =
                std::max(state_lowerbound_(i % state_dim_), std::min(state_upperbound_(i % state_dim_), optimal_solution_(i)));
        }
        for (int i = 0; i < nu_; ++i) {
            optimal_solution_(nx_ + i) = std::max(input_lowerbound_(i % input_dim_),
                                                  std::min(input_upperbound_(i % input_dim_), optimal_solution_(nx_ + i)));
        }
    }

    update_trajectory_data();
}

void TrajectoryOptimizer::update_trajectory_data() {
    path_x_.clear();
    path_y_.clear();
    path_yaw_.clear();
    velocity_.clear();
    steering_angle_.clear();
    acceleration_.clear();
    steering_rate_.clear();

    for (int i = 0; i <= prediction_horizon_; ++i) {
        path_x_.push_back(optimal_solution_(state_dim_ * i));
        path_y_.push_back(optimal_solution_(state_dim_ * i + 1));
        path_yaw_.push_back(optimal_solution_(state_dim_ * i + 2));
        velocity_.push_back(optimal_solution_(state_dim_ * i + 3));
        steering_angle_.push_back(optimal_solution_(state_dim_ * i + 4));

        if (i < prediction_horizon_) {
            acceleration_.push_back(optimal_solution_(nx_ + input_dim_ * i));
            steering_rate_.push_back(optimal_solution_(nx_ + input_dim_ * i + 1));
        }
    }
}

QPData TrajectoryOptimizer::setup_qp(const SystemModel& system_model,
                                     Eigen::Matrix<double, 5, 5>& q,
                                     Eigen::Matrix<double, 2, 2>& r) {
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(total_vars_all_slack_, total_vars_all_slack_);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(total_vars_all_slack_);

    Eigen::MatrixXd ceq = Eigen::MatrixXd::Zero(n_eq_, total_vars_all_slack_);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq_);

    Eigen::MatrixXd cineq = Eigen::MatrixXd::Zero(n_ineq_, total_vars_all_slack_);
    Eigen::VectorXd bineq_lower = Eigen::VectorXd::Zero(n_ineq_);
    Eigen::VectorXd bineq_upper = Eigen::VectorXd::Zero(n_ineq_);

    // Hessian setup (cost function)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        h.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) = q;
        h.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) = r;
    }
    h.block(nx_ - state_dim_, nx_ - state_dim_, state_dim_, state_dim_) = q;

    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        f.segment(state_dim_ * time_step, state_dim_) = q * optimal_solution_.segment(state_dim_ * time_step, state_dim_);
        f.segment(nx_ + input_dim_ * time_step, input_dim_) =
            r * optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_);
    }
    f.segment(nx_ - state_dim_, state_dim_) = q * optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    // Slack penalties
    h.block(total_vars_, total_vars_, n_slack_, n_slack_) = rho_goal_ * Eigen::MatrixXd::Identity(n_slack_, n_slack_);
    h.block(total_vars_ + n_slack_, total_vars_ + n_slack_, n_obstacle_slack_, n_obstacle_slack_) =
        rho_obs_ * Eigen::MatrixXd::Identity(n_obstacle_slack_, n_obstacle_slack_);

    // Equality constraints (dynamics and initial/goal)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        SystemState xk_next(optimal_solution_.segment(state_dim_ * (time_step + 1), state_dim_));
        SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));

        auto [Ak, Bk, gk] = system_model.get_system_jacobian(xk, uk, ts_);

        ceq.block(state_dim_ * (time_step + 1), state_dim_ * (time_step + 1), state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        ceq.block(state_dim_ * (time_step + 1), state_dim_ * time_step, state_dim_, state_dim_) = -Ak;
        ceq.block(state_dim_ * (time_step + 1), nx_ + input_dim_ * time_step, state_dim_, input_dim_) = -Bk;
        beq.segment(state_dim_ * (time_step + 1), state_dim_) = (Ak * xk() + Bk * uk() + gk) - xk_next();
    }
    ceq.block(0, 0, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    ceq.block(n_eq_ - state_dim_, nx_ - state_dim_, state_dim_, state_dim_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    ceq.block(n_eq_ - state_dim_, total_vars_, state_dim_, n_slack_) = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    beq.segment(n_eq_ - state_dim_, state_dim_) = x_goal_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    // Inequality constraints (state/input bounds)
    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        cineq.block(state_dim_ * time_step, state_dim_ * time_step, state_dim_, state_dim_) =
            Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        bineq_lower.segment(state_dim_ * time_step, state_dim_) = state_lowerbound_ - xk();
        bineq_upper.segment(state_dim_ * time_step, state_dim_) = state_upperbound_ - xk();
        if (time_step < prediction_horizon_) {
            SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));
            cineq.block(nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, input_dim_, input_dim_) =
                Eigen::MatrixXd::Identity(input_dim_, input_dim_);
            bineq_lower.segment(nx_ + input_dim_ * time_step, input_dim_) = input_lowerbound_ - uk();
            bineq_upper.segment(nx_ + input_dim_ * time_step, input_dim_) = input_upperbound_ - uk();
        }
    }

    Eigen::MatrixXd cobs = Eigen::MatrixXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_, total_vars_all_slack_);
    Eigen::VectorXd bobs_lower = Eigen::VectorXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_);
    Eigen::VectorXd bobs_upper = Eigen::VectorXd::Zero(n_obstacle_constraints_ + n_obstacle_slack_);

    Eigen::Index constraint_idx = 0;
    Eigen::Index slack_idx = total_vars_ + n_slack_;

    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        double x_k = optimal_solution_(state_dim_ * time_step);
        double y_k = optimal_solution_(state_dim_ * time_step + 1);

        for (const Obstacle& obs : obstacles_) {
            auto [c_x, c_y] = Utils::find_closest_point_on_obstacle(x_k, y_k, obs);
            double d_safe = vehicle_radius_ + safety_margin_;

            double dx = x_k - c_x;
            double dy = y_k - c_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            double grad_x = dist > 1e-4 ? dx / dist : 0.0;
            double grad_y = dist > 1e-4 ? dy / dist : 0.0;

            cobs(constraint_idx, state_dim_ * time_step) = grad_x;
            cobs(constraint_idx, state_dim_ * time_step + 1) = grad_y;
            cobs(constraint_idx, slack_idx) = 1.0;
            cobs(n_obstacle_constraints_ + constraint_idx, slack_idx) = 1.0;
            bobs_lower(constraint_idx) = d_safe - dist;
            bobs_upper(constraint_idx) = std::numeric_limits<double>::infinity();
            bobs_lower(n_obstacle_constraints_ + constraint_idx) = 0.0;
            bobs_upper(n_obstacle_constraints_ + constraint_idx) = std::numeric_limits<double>::infinity();

            constraint_idx++;
            slack_idx++;
        }
    }

    // Combine all constraints
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(total_constraints_, total_vars_all_slack_);
    Eigen::VectorXd lower_bound(total_constraints_);
    Eigen::VectorXd upper_bound(total_constraints_);

    a.block(0, 0, n_eq_, total_vars_all_slack_) = ceq;
    a.block(n_eq_, 0, n_ineq_, total_vars_all_slack_) = cineq;
    a.block(n_eq_ + n_ineq_, 0, n_obstacle_constraints_ + n_obstacle_slack_, total_vars_all_slack_) = cobs;

    lower_bound.head(n_eq_) = beq;
    upper_bound.head(n_eq_) = beq;
    lower_bound.segment(n_eq_, n_ineq_) = bineq_lower;
    upper_bound.segment(n_eq_, n_ineq_) = bineq_upper;
    lower_bound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_lower;
    upper_bound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_upper;

    return {h, f, a, lower_bound, upper_bound};
}

}  // namespace optimal_parking