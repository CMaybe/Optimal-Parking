#include "optimal_parking/trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "optimal_parking/config.hpp"
#include "optimal_parking/qp_solver.hpp"
#include "optimal_parking/rrt_star.hpp"
#include "optimal_parking/utils.hpp"

namespace optimal_parking {
TrajectoryOptimizer::TrajectoryOptimizer(const std::string& config_path) {
    const PlannerConfig config = load_planner_config(config_path);

    trajectory_time_ = config.trajectory_time;
    ts_ = config.ts;
    state_lowerbound_ = config.state_lower_bound;
    state_upperbound_ = config.state_upper_bound;
    input_lowerbound_ = config.input_lower_bound;
    input_upperbound_ = config.input_upper_bound;
    n_sqp_ = config.sqp_iterations;
    qp_iteration_ = config.qp_iterations;
    rho_goal_ = config.goal_penalty;
    rho_obs_ = config.obstacle_penalty;
    safety_margin_ = config.safety_margin;
    obstacles_ = config.obstacles;

    rrt_star_ = std::make_unique<RRTStar>(obstacles_,
                                          config.map_x_min,
                                          config.map_x_max,
                                          config.map_y_min,
                                          config.map_y_max,
                                          config.goal_radius,
                                          config.goal_bias,
                                          config.step_distance,
                                          config.rewire_radius,
                                          config.rrt_iterations,
                                          config.vehicle_length,
                                          config.vehicle_width);

    q_ = config.state_weight.asDiagonal();
    r_ = config.input_weight.asDiagonal();

    prediction_horizon_ = static_cast<Eigen::Index>(std::ceil(trajectory_time_ / ts_));
    state_dim_ = 5;
    input_dim_ = 2;

    update_problem_dimensions();

    x_goal_.setZero();
    x0_.setZero();
    optimal_solution_.setZero(total_vars_);
}

void TrajectoryOptimizer::set_obstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
    rrt_star_->set_obstacles(obstacles_);
    update_problem_dimensions();
}

void TrajectoryOptimizer::update_problem_dimensions() {
    n_obstacle_constraints_ = (prediction_horizon_ + 1) * static_cast<Eigen::Index>(obstacles_.size());
    n_obstacle_slack_ = n_obstacle_constraints_;
    total_vars_all_slack_ = total_vars_ + n_slack_ + n_obstacle_slack_;
    total_constraints_ = n_eq_ + n_ineq_ + n_obstacle_constraints_ + n_obstacle_slack_;
}

void TrajectoryOptimizer::set_goal_pose(const Eigen::Vector<double, 5>& goal_pose) { x_goal_ = goal_pose; }
void TrajectoryOptimizer::set_initial_pose(const Eigen::Vector<double, 5>& initial_pose) { x0_ = initial_pose; }

void TrajectoryOptimizer::run_sqp(const SystemModel& system_model) {
    optimal_solution_.setZero(total_vars_);
    vehicle_radius_ = 0.5 * std::hypot(system_model.vehicle_length(), system_model.vehicle_width());

    Eigen::Vector3d start(x0_(0), x0_(1), x0_(2));
    Eigen::Vector3d goal(x_goal_(0), x_goal_(1), x_goal_(2));
    std::vector<Eigen::Vector3d> path = rrt_star_->make_path(start, goal, prediction_horizon_ + 1);
    if (path.size() == static_cast<std::size_t>(prediction_horizon_ + 1)) {
        optimal_solution_.segment(0, state_dim_) = x0_;
        for (int i = 0; i <= prediction_horizon_; ++i) {
            optimal_solution_(state_dim_ * i) = path[i](0);
            optimal_solution_(state_dim_ * i + 1) = path[i](1);
            optimal_solution_(state_dim_ * i + 2) = path[i](2);
        }
    } else {
        std::cerr << "RRT* path planning failed to produce the requested horizon." << std::endl;
        return;
    }
    for (int iter = 0; iter < n_sqp_; ++iter) {
        auto [hessian, gradient, linearMatrix, lowerBound, upperBound] = setup_qp(system_model, q_, r_);

        const auto solution = solve_qp({hessian, gradient, linearMatrix, lowerBound, upperBound},
                                       {.max_iterations = qp_iteration_,
                                        .absolute_tolerance = 1e-3,
                                        .relative_tolerance = 1e-3,
                                        .warm_start = true,
                                        .verbose = false});
        if (!solution) {
            std::cerr << "QP failed at iteration " << iter << std::endl;
            return;
        }

        const Eigen::VectorXd& delta_solution = *solution;
        const Eigen::VectorXd delta_variables = delta_solution.head(total_vars_);
        std::cout << "Iteration " << iter << ": delta_solution norm = " << delta_variables.norm() << std::endl;

        if (delta_variables.norm() < 0.05) {
            std::cout << "Delta solution norm: " << delta_variables.norm() << "\nConverged at iteration " << iter << "\n";
            break;
        }

        const auto merit = [](const Eigen::VectorXd& step,
                              const Eigen::SparseMatrix<double>& hessian_matrix,
                              const Eigen::VectorXd& gradient_vector,
                              const Eigen::SparseMatrix<double>& constraint_matrix,
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
    const auto state_count = static_cast<std::size_t>(prediction_horizon_ + 1);
    const auto input_count = static_cast<std::size_t>(prediction_horizon_);
    path_x_.resize(state_count);
    path_y_.resize(state_count);
    path_yaw_.resize(state_count);
    velocity_.resize(state_count);
    steering_angle_.resize(state_count);
    acceleration_.resize(input_count);
    steering_rate_.resize(input_count);

    for (int i = 0; i <= prediction_horizon_; ++i) {
        path_x_[i] = optimal_solution_(state_dim_ * i);
        path_y_[i] = optimal_solution_(state_dim_ * i + 1);
        path_yaw_[i] = optimal_solution_(state_dim_ * i + 2);
        velocity_[i] = optimal_solution_(state_dim_ * i + 3);
        steering_angle_[i] = optimal_solution_(state_dim_ * i + 4);

        if (i < prediction_horizon_) {
            acceleration_[i] = optimal_solution_(nx_ + input_dim_ * i);
            steering_rate_[i] = optimal_solution_(nx_ + input_dim_ * i + 1);
        }
    }
}

QPData TrajectoryOptimizer::setup_qp(const SystemModel& system_model,
                                     const Eigen::Matrix<double, 5, 5>& q,
                                     const Eigen::Matrix<double, 2, 2>& r) {
    std::vector<Eigen::Triplet<double>> hessian_triplets;
    std::vector<Eigen::Triplet<double>> constraint_triplets;
    const auto add_block = [](std::vector<Eigen::Triplet<double>>& triplets,
                              Eigen::Index row_offset,
                              Eigen::Index column_offset,
                              const auto& block) {
        for (Eigen::Index row = 0; row < block.rows(); ++row) {
            for (Eigen::Index column = 0; column < block.cols(); ++column) {
                if (block(row, column) != 0.0) {
                    triplets.emplace_back(row_offset + row, column_offset + column, block(row, column));
                }
            }
        }
    };

    Eigen::VectorXd f = Eigen::VectorXd::Zero(total_vars_all_slack_);

    Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq_);

    Eigen::VectorXd bineq_lower = Eigen::VectorXd::Zero(n_ineq_);
    Eigen::VectorXd bineq_upper = Eigen::VectorXd::Zero(n_ineq_);

    // Hessian setup (cost function)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        add_block(hessian_triplets, state_dim_ * time_step, state_dim_ * time_step, q);
        add_block(hessian_triplets, nx_ + input_dim_ * time_step, nx_ + input_dim_ * time_step, r);
    }
    add_block(hessian_triplets, nx_ - state_dim_, nx_ - state_dim_, q);

    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        f.segment(state_dim_ * time_step, state_dim_) = q * optimal_solution_.segment(state_dim_ * time_step, state_dim_);
        f.segment(nx_ + input_dim_ * time_step, input_dim_) =
            r * optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_);
    }
    f.segment(nx_ - state_dim_, state_dim_) = q * optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    // Slack penalties
    add_block(hessian_triplets, total_vars_, total_vars_, rho_goal_ * Eigen::MatrixXd::Identity(n_slack_, n_slack_));
    add_block(hessian_triplets,
              total_vars_ + n_slack_,
              total_vars_ + n_slack_,
              rho_obs_ * Eigen::MatrixXd::Identity(n_obstacle_slack_, n_obstacle_slack_));

    // Equality constraints (dynamics and initial/goal)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        SystemState xk_next(optimal_solution_.segment(state_dim_ * (time_step + 1), state_dim_));
        SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));

        auto [Ak, Bk, gk] = system_model.get_system_jacobian(xk, uk, ts_);

        const Eigen::Index equality_row = state_dim_ * (time_step + 1);
        add_block(constraint_triplets, equality_row, state_dim_ * (time_step + 1), Eigen::MatrixXd::Identity(state_dim_, state_dim_));
        add_block(constraint_triplets, equality_row, state_dim_ * time_step, -Ak);
        add_block(constraint_triplets, equality_row, nx_ + input_dim_ * time_step, -Bk);
        beq.segment(state_dim_ * (time_step + 1), state_dim_) = (Ak * xk() + Bk * uk() + gk) - xk_next();
    }
    add_block(constraint_triplets, 0, 0, Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    add_block(constraint_triplets, n_eq_ - state_dim_, nx_ - state_dim_, Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    add_block(constraint_triplets, n_eq_ - state_dim_, total_vars_, Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    beq.segment(n_eq_ - state_dim_, state_dim_) = x_goal_ - optimal_solution_.segment(nx_ - state_dim_, state_dim_);

    // Inequality constraints (state/input bounds)
    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        SystemState xk(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        add_block(constraint_triplets,
                  n_eq_ + state_dim_ * time_step,
                  state_dim_ * time_step,
                  Eigen::MatrixXd::Identity(state_dim_, state_dim_));
        bineq_lower.segment(state_dim_ * time_step, state_dim_) = state_lowerbound_ - xk();
        bineq_upper.segment(state_dim_ * time_step, state_dim_) = state_upperbound_ - xk();
        if (time_step < prediction_horizon_) {
            SystemInput uk(optimal_solution_.segment(nx_ + input_dim_ * time_step, input_dim_));
            add_block(constraint_triplets,
                      n_eq_ + nx_ + input_dim_ * time_step,
                      nx_ + input_dim_ * time_step,
                      Eigen::MatrixXd::Identity(input_dim_, input_dim_));
            bineq_lower.segment(nx_ + input_dim_ * time_step, input_dim_) = input_lowerbound_ - uk();
            bineq_upper.segment(nx_ + input_dim_ * time_step, input_dim_) = input_upperbound_ - uk();
        }
    }

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

            const Eigen::Index obstacle_row = n_eq_ + n_ineq_ + constraint_idx;
            const Eigen::Index slack_row = obstacle_row + n_obstacle_constraints_;
            constraint_triplets.emplace_back(obstacle_row, state_dim_ * time_step, grad_x);
            constraint_triplets.emplace_back(obstacle_row, state_dim_ * time_step + 1, grad_y);
            constraint_triplets.emplace_back(obstacle_row, slack_idx, 1.0);
            constraint_triplets.emplace_back(slack_row, slack_idx, 1.0);
            bobs_lower(constraint_idx) = d_safe - dist;
            bobs_upper(constraint_idx) = std::numeric_limits<double>::infinity();
            bobs_lower(n_obstacle_constraints_ + constraint_idx) = 0.0;
            bobs_upper(n_obstacle_constraints_ + constraint_idx) = std::numeric_limits<double>::infinity();

            constraint_idx++;
            slack_idx++;
        }
    }

    Eigen::SparseMatrix<double> h(total_vars_all_slack_, total_vars_all_slack_);
    h.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    Eigen::SparseMatrix<double> a(total_constraints_, total_vars_all_slack_);
    a.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
    Eigen::VectorXd lower_bound(total_constraints_);
    Eigen::VectorXd upper_bound(total_constraints_);

    lower_bound.head(n_eq_) = beq;
    upper_bound.head(n_eq_) = beq;
    lower_bound.segment(n_eq_, n_ineq_) = bineq_lower;
    upper_bound.segment(n_eq_, n_ineq_) = bineq_upper;
    lower_bound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_lower;
    upper_bound.segment(n_eq_ + n_ineq_, n_obstacle_constraints_ + n_obstacle_slack_) = bobs_upper;

    return {h, f, a, lower_bound, upper_bound};
}

}  // namespace optimal_parking