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
    void run_sqp(const SystemModel& system_model);
    void update_trajectory_data();
    QPData setup_qp(const SystemModel& system_model,
                   const Eigen::Matrix<double, 5, 5>& q,
                   const Eigen::Matrix<double, 2, 2>& r);

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
    double ts_;
    int n_sqp_;
    int qp_iteration_;
    double rho_goal_, rho_obs_;
    Eigen::Vector<double, 5> state_lowerbound_, state_upperbound_;
    Eigen::Vector<double, 2> input_lowerbound_, input_upperbound_;
    std::vector<Obstacle> obstacles_;
    double safety_margin_;

    Eigen::VectorXd optimal_solution_;

    Eigen::Vector<double, 5> x0_;
    Eigen::Vector<double, 5> x_goal_;
    Eigen::Matrix<double, 5, 5> q_;
    Eigen::Matrix<double, 2, 2> r_;

    Eigen::Index prediction_horizon_;

    Eigen::Index state_dim_;
    Eigen::Index input_dim_;

    Eigen::Index nx_, nu_, total_vars_;
    Eigen::Index n_eq_;
    Eigen::Index n_ineq_;

    Eigen::Index n_slack_;
    Eigen::Index n_obstacle_constraints_;
    Eigen::Index n_obstacle_slack_;
    Eigen::Index total_vars_all_slack_;
    Eigen::Index total_constraints_;

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