#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <vector>

#include "optimal_parking/rrt_star.hpp"
#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_model.hpp"
#include "optimal_parking/system/system_state.hpp"
#include "optimal_parking/types.hpp"

namespace optimal_parking {

class TrajectoryOptimizer {
public:
    TrajectoryOptimizer(const std::string& config_path);
    void set_goal_pose(const Eigen::Vector<double, 5>& goal_pose);
    void set_initial_pose(const Eigen::Vector<double, 5>& initial_pose);
    void set_obstacles(const std::vector<Obstacle>& obstacles);
    void set_safety_margin(double safety_margin) { safety_margin_ = safety_margin; }
    void set_goal_penalty_weight(double goal_penalty_weight) { goal_penalty_weight_ = goal_penalty_weight; }
    void set_obstacle_penalty_weight(double obstacle_penalty_weight) { obstacle_penalty_weight_ = obstacle_penalty_weight; }
    void set_max_sqp_iterations(int max_sqp_iterations) { max_sqp_iterations_ = max_sqp_iterations; }
    void set_max_qp_iterations(int max_qp_iterations) { max_qp_iterations_ = max_qp_iterations; }
    void run_sqp(const SystemModel& system_model);
    void update_trajectory_data();
    QPData setup_qp(const SystemModel& system_model,
                    const Eigen::Matrix<double, 5, 5>& state_weight_matrix,
                    const Eigen::Matrix<double, 2, 2>& input_weight_matrix);

    [[nodiscard]] TrajectoryData get_trajectory_data() const {
        return {path_x_, path_y_, path_yaw_, velocity_, steering_angle_, acceleration_, steering_rate_};
    }
    [[nodiscard]] inline const Eigen::VectorXd& get_optimal_solution() const { return optimal_solution_; }
    [[nodiscard]] inline const std::vector<double>& get_path_x() const { return path_x_; }
    [[nodiscard]] inline const std::vector<double>& get_path_y() const { return path_y_; }
    [[nodiscard]] inline const std::vector<double>& get_path_yaw() const { return path_yaw_; }

private:
    void update_problem_dimensions();

    double trajectory_time_;
    double sample_time_;
    int max_sqp_iterations_;
    int max_qp_iterations_;
    double goal_penalty_weight_, obstacle_penalty_weight_;
    Eigen::Vector<double, 5> state_lower_bound_, state_upper_bound_;
    Eigen::Vector<double, 2> input_lower_bound_, input_upper_bound_;
    std::vector<Obstacle> obstacles_;
    double safety_margin_;

    Eigen::VectorXd optimal_solution_;

    Eigen::Vector<double, 5> initial_state_;
    Eigen::Vector<double, 5> goal_state_;
    Eigen::Matrix<double, 5, 5> state_weight_matrix_;
    Eigen::Matrix<double, 2, 2> input_weight_matrix_;

    Eigen::Index prediction_horizon_;

    static constexpr Eigen::Index kStateDim = 5;
    static constexpr Eigen::Index kInputDim = 2;

    Eigen::Index num_state_variables_, num_input_variables_, num_decision_variables_;
    Eigen::Index num_equality_constraints_;
    Eigen::Index num_inequality_constraints_;

    Eigen::Index num_slack_variables_;
    Eigen::Index num_obstacle_constraints_;
    Eigen::Index num_obstacle_slack_variables_;
    Eigen::Index num_total_variables_with_slack_;
    Eigen::Index num_total_constraints_;

    double vehicle_radius_;

    std::vector<double> path_x_;
    std::vector<double> path_y_;
    std::vector<double> path_yaw_;
    std::vector<double> velocity_;
    std::vector<double> steering_angle_;
    std::vector<double> acceleration_;
    std::vector<double> steering_rate_;

    std::unique_ptr<RRTStar> rrt_star_;
};
}  // namespace optimal_parking
#endif  // TRAJECTORY_OPTIMIZER_HPP