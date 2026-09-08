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
    sample_time_ = config.ts;
    state_lower_bound_ = config.state_lower_bound;
    state_upper_bound_ = config.state_upper_bound;
    input_lower_bound_ = config.input_lower_bound;
    input_upper_bound_ = config.input_upper_bound;
    max_sqp_iterations_ = config.sqp_iterations;
    max_qp_iterations_ = config.qp_iterations;
    goal_penalty_weight_ = config.goal_penalty;
    obstacle_penalty_weight_ = config.obstacle_penalty;
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

    state_weight_matrix_ = config.state_weight.asDiagonal();
    input_weight_matrix_ = config.input_weight.asDiagonal();

    prediction_horizon_ = static_cast<Eigen::Index>(std::ceil(trajectory_time_ / sample_time_));
    state_dim_ = 5;
    input_dim_ = 2;

    update_problem_dimensions();

    goal_state_.setZero();
    initial_state_.setZero();
    optimal_solution_.setZero(num_decision_variables_);
}

void TrajectoryOptimizer::set_obstacles(const std::vector<Obstacle>& obstacles) {
    obstacles_ = obstacles;
    rrt_star_->set_obstacles(obstacles_);
    update_problem_dimensions();
}

void TrajectoryOptimizer::update_problem_dimensions() {
    num_state_variables_ = state_dim_ * (prediction_horizon_ + 1);
    num_input_variables_ = input_dim_ * prediction_horizon_;
    num_decision_variables_ = num_state_variables_ + num_input_variables_;
    num_equality_constraints_ = state_dim_ * (prediction_horizon_ + 1) + state_dim_;
    num_inequality_constraints_ = num_decision_variables_;
    num_slack_variables_ = state_dim_;
    num_obstacle_constraints_ = (prediction_horizon_ + 1) * static_cast<Eigen::Index>(obstacles_.size());
    num_obstacle_slack_variables_ = num_obstacle_constraints_;
    num_total_variables_with_slack_ = num_decision_variables_ + num_slack_variables_ + num_obstacle_slack_variables_;
    num_total_constraints_ =
        num_equality_constraints_ + num_inequality_constraints_ + num_obstacle_constraints_ + num_obstacle_slack_variables_;
}

void TrajectoryOptimizer::set_goal_pose(const Eigen::Vector<double, 5>& goal_pose) { goal_state_ = goal_pose; }
void TrajectoryOptimizer::set_initial_pose(const Eigen::Vector<double, 5>& initial_pose) { initial_state_ = initial_pose; }

void TrajectoryOptimizer::run_sqp(const SystemModel& system_model) {
    optimal_solution_.setZero(num_decision_variables_);
    vehicle_radius_ = 0.5 * std::hypot(system_model.vehicle_length(), system_model.vehicle_width());

    Eigen::Vector3d start(initial_state_(0), initial_state_(1), initial_state_(2));
    Eigen::Vector3d goal(goal_state_(0), goal_state_(1), goal_state_(2));
    std::vector<Eigen::Vector3d> path = rrt_star_->make_path(start, goal, prediction_horizon_ + 1);
    if (path.size() == static_cast<std::size_t>(prediction_horizon_ + 1)) {
        optimal_solution_.segment(0, state_dim_) = initial_state_;
        for (int i = 0; i <= prediction_horizon_; ++i) {
            optimal_solution_(state_dim_ * i) = path[i](0);
            optimal_solution_(state_dim_ * i + 1) = path[i](1);
            optimal_solution_(state_dim_ * i + 2) = path[i](2);
        }
    } else {
        std::cerr << "RRT* path planning failed to produce the requested horizon." << std::endl;
        return;
    }
    for (int iter = 0; iter < max_sqp_iterations_; ++iter) {
        auto [hessian, gradient, linear_matrix, lower_bound, upper_bound] =
            setup_qp(system_model, state_weight_matrix_, input_weight_matrix_);

        const auto solution = solve_qp({hessian, gradient, linear_matrix, lower_bound, upper_bound},
                                       {.max_iterations = max_qp_iterations_,
                                        .absolute_tolerance = 1e-3,
                                        .relative_tolerance = 1e-3,
                                        .warm_start = true,
                                        .verbose = false});
        if (!solution) {
            std::cerr << "QP failed at iteration " << iter << std::endl;
            return;
        }

        const Eigen::VectorXd& delta_solution = *solution;
        const Eigen::VectorXd delta_variables = delta_solution.head(num_decision_variables_);
        const double mean_delta = delta_variables.norm() / std::sqrt(static_cast<double>(num_decision_variables_));
        const double max_delta = delta_variables.lpNorm<Eigen::Infinity>();
        std::cout << "Iteration " << iter << ": max_delta = " << max_delta << ", mean_delta = " << mean_delta << std::endl;

        if (max_delta < 5e-2 || mean_delta < 1e-2) {
            std::cout << "Converged at iteration " << iter << "\n";
            break;
        }

        const Eigen::VectorXd a_delta = linear_matrix * delta_solution;
        const Eigen::VectorXd h_delta = hessian * delta_solution;
        const double quad_coeff = 0.5 * delta_solution.dot(h_delta);
        const double lin_coeff = gradient.dot(delta_solution);

        const auto merit = [](double s,
                              double quad,
                              double lin,
                              const Eigen::VectorXd& ax,
                              const Eigen::VectorXd& lb,
                              const Eigen::VectorXd& ub) {
            double violation = 0.0;
            for (Eigen::Index i = 0; i < ax.size(); ++i) {
                const double val = s * ax(i);
                violation += std::max(lb(i) - val, 0.0);
                violation += std::max(val - ub(i), 0.0);
            }
            return s * s * quad + s * lin + 1e4 * violation;
        };

        const double current_merit = merit(0.0, quad_coeff, lin_coeff, a_delta, lower_bound, upper_bound);
        double step_length = 1.0;
        while (step_length > 1e-3 &&
               merit(step_length, quad_coeff, lin_coeff, a_delta, lower_bound, upper_bound) > current_merit) {
            step_length *= 0.5;
        }
        optimal_solution_ += step_length * delta_variables;
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
            acceleration_[i] = optimal_solution_(num_state_variables_ + input_dim_ * i);
            steering_rate_[i] = optimal_solution_(num_state_variables_ + input_dim_ * i + 1);
        }
    }
}

QPData TrajectoryOptimizer::setup_qp(const SystemModel& system_model,
                                     const Eigen::Matrix<double, 5, 5>& state_weight_matrix,
                                     const Eigen::Matrix<double, 2, 2>& input_weight_matrix) {
    std::vector<Eigen::Triplet<double>> hessian_triplets;
    std::vector<Eigen::Triplet<double>> constraint_triplets;
    hessian_triplets.reserve(static_cast<std::size_t>(num_total_variables_with_slack_));
    constraint_triplets.reserve(
        static_cast<std::size_t>(num_equality_constraints_ * 4 + num_inequality_constraints_ + num_obstacle_constraints_ * 4));

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

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_total_variables_with_slack_);

    Eigen::VectorXd equality_vector = Eigen::VectorXd::Zero(num_equality_constraints_);

    Eigen::VectorXd inequality_lower_bound = Eigen::VectorXd::Zero(num_inequality_constraints_);
    Eigen::VectorXd inequality_upper_bound = Eigen::VectorXd::Zero(num_inequality_constraints_);

    // Hessian setup (cost function)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        add_block(hessian_triplets, state_dim_ * time_step, state_dim_ * time_step, state_weight_matrix);
        add_block(hessian_triplets,
                  num_state_variables_ + input_dim_ * time_step,
                  num_state_variables_ + input_dim_ * time_step,
                  input_weight_matrix);
    }
    add_block(hessian_triplets, num_state_variables_ - state_dim_, num_state_variables_ - state_dim_, state_weight_matrix);

    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        gradient.segment(state_dim_ * time_step, state_dim_) =
            state_weight_matrix * optimal_solution_.segment(state_dim_ * time_step, state_dim_);
        gradient.segment(num_state_variables_ + input_dim_ * time_step, input_dim_) =
            input_weight_matrix * optimal_solution_.segment(num_state_variables_ + input_dim_ * time_step, input_dim_);
    }
    gradient.segment(num_state_variables_ - state_dim_, state_dim_) =
        state_weight_matrix * optimal_solution_.segment(num_state_variables_ - state_dim_, state_dim_);

    // Slack penalties
    add_block(hessian_triplets,
              num_decision_variables_,
              num_decision_variables_,
              goal_penalty_weight_ * Eigen::MatrixXd::Identity(num_slack_variables_, num_slack_variables_));
    add_block(hessian_triplets,
              num_decision_variables_ + num_slack_variables_,
              num_decision_variables_ + num_slack_variables_,
              obstacle_penalty_weight_ * Eigen::MatrixXd::Identity(num_obstacle_slack_variables_, num_obstacle_slack_variables_));

    // Equality constraints (dynamics and initial/goal)
    for (int time_step = 0; time_step < prediction_horizon_; ++time_step) {
        SystemState current_state(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        SystemState next_state(optimal_solution_.segment(state_dim_ * (time_step + 1), state_dim_));
        SystemInput current_input(optimal_solution_.segment(num_state_variables_ + input_dim_ * time_step, input_dim_));

        auto [discrete_a, discrete_b, discrete_g] =
            system_model.compute_discrete_linearization(current_state, current_input, sample_time_);

        const Eigen::Index equality_row = state_dim_ * (time_step + 1);
        add_block(
            constraint_triplets, equality_row, state_dim_ * (time_step + 1), Eigen::MatrixXd::Identity(state_dim_, state_dim_));
        add_block(constraint_triplets, equality_row, state_dim_ * time_step, -discrete_a);
        add_block(constraint_triplets, equality_row, num_state_variables_ + input_dim_ * time_step, -discrete_b);
        equality_vector.segment(state_dim_ * (time_step + 1), state_dim_) =
            (discrete_a * current_state() + discrete_b * current_input() + discrete_g) - next_state();
    }
    add_block(constraint_triplets, 0, 0, Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    add_block(constraint_triplets,
              num_equality_constraints_ - state_dim_,
              num_state_variables_ - state_dim_,
              Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    add_block(constraint_triplets,
              num_equality_constraints_ - state_dim_,
              num_decision_variables_,
              Eigen::MatrixXd::Identity(state_dim_, state_dim_));
    equality_vector.segment(num_equality_constraints_ - state_dim_, state_dim_) =
        goal_state_ - optimal_solution_.segment(num_state_variables_ - state_dim_, state_dim_);

    // Inequality constraints (state/input bounds)
    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        SystemState current_state(optimal_solution_.segment(state_dim_ * time_step, state_dim_));
        add_block(constraint_triplets,
                  num_equality_constraints_ + state_dim_ * time_step,
                  state_dim_ * time_step,
                  Eigen::MatrixXd::Identity(state_dim_, state_dim_));
        inequality_lower_bound.segment(state_dim_ * time_step, state_dim_) = state_lower_bound_ - current_state();
        inequality_upper_bound.segment(state_dim_ * time_step, state_dim_) = state_upper_bound_ - current_state();
        if (time_step < prediction_horizon_) {
            SystemInput current_input(optimal_solution_.segment(num_state_variables_ + input_dim_ * time_step, input_dim_));
            add_block(constraint_triplets,
                      num_equality_constraints_ + num_state_variables_ + input_dim_ * time_step,
                      num_state_variables_ + input_dim_ * time_step,
                      Eigen::MatrixXd::Identity(input_dim_, input_dim_));
            inequality_lower_bound.segment(num_state_variables_ + input_dim_ * time_step, input_dim_) =
                input_lower_bound_ - current_input();
            inequality_upper_bound.segment(num_state_variables_ + input_dim_ * time_step, input_dim_) =
                input_upper_bound_ - current_input();
        }
    }

    Eigen::VectorXd obstacle_lower_bound = Eigen::VectorXd::Zero(num_obstacle_constraints_ + num_obstacle_slack_variables_);
    Eigen::VectorXd obstacle_upper_bound = Eigen::VectorXd::Zero(num_obstacle_constraints_ + num_obstacle_slack_variables_);

    Eigen::Index constraint_idx = 0;
    Eigen::Index slack_idx = num_decision_variables_ + num_slack_variables_;

    for (int time_step = 0; time_step <= prediction_horizon_; ++time_step) {
        double current_x = optimal_solution_(state_dim_ * time_step);
        double current_y = optimal_solution_(state_dim_ * time_step + 1);

        for (const Obstacle& obstacle : obstacles_) {
            auto [closest_x, closest_y] = Utils::find_closest_point_on_obstacle(current_x, current_y, obstacle);
            double safety_distance = vehicle_radius_ + safety_margin_;

            double dx = current_x - closest_x;
            double dy = current_y - closest_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            double grad_x = dist > 1e-4 ? dx / dist : 0.0;
            double grad_y = dist > 1e-4 ? dy / dist : 0.0;

            const Eigen::Index obstacle_row = num_equality_constraints_ + num_inequality_constraints_ + constraint_idx;
            const Eigen::Index slack_row = obstacle_row + num_obstacle_constraints_;
            constraint_triplets.emplace_back(obstacle_row, state_dim_ * time_step, grad_x);
            constraint_triplets.emplace_back(obstacle_row, state_dim_ * time_step + 1, grad_y);
            constraint_triplets.emplace_back(obstacle_row, slack_idx, 1.0);
            constraint_triplets.emplace_back(slack_row, slack_idx, 1.0);
            obstacle_lower_bound(constraint_idx) = safety_distance - dist;
            obstacle_upper_bound(constraint_idx) = std::numeric_limits<double>::infinity();
            obstacle_lower_bound(num_obstacle_constraints_ + constraint_idx) = 0.0;
            obstacle_upper_bound(num_obstacle_constraints_ + constraint_idx) = std::numeric_limits<double>::infinity();

            constraint_idx++;
            slack_idx++;
        }
    }

    Eigen::SparseMatrix<double> hessian(num_total_variables_with_slack_, num_total_variables_with_slack_);
    hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    Eigen::SparseMatrix<double> constraint_matrix(num_total_constraints_, num_total_variables_with_slack_);
    constraint_matrix.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
    Eigen::VectorXd lower_bound(num_total_constraints_);
    Eigen::VectorXd upper_bound(num_total_constraints_);

    lower_bound.head(num_equality_constraints_) = equality_vector;
    upper_bound.head(num_equality_constraints_) = equality_vector;
    lower_bound.segment(num_equality_constraints_, num_inequality_constraints_) = inequality_lower_bound;
    upper_bound.segment(num_equality_constraints_, num_inequality_constraints_) = inequality_upper_bound;
    lower_bound.segment(num_equality_constraints_ + num_inequality_constraints_,
                        num_obstacle_constraints_ + num_obstacle_slack_variables_) = obstacle_lower_bound;
    upper_bound.segment(num_equality_constraints_ + num_inequality_constraints_,
                        num_obstacle_constraints_ + num_obstacle_slack_variables_) = obstacle_upper_bound;

    return {hessian, gradient, constraint_matrix, lower_bound, upper_bound};
}

}  // namespace optimal_parking