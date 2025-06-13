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
    void setGoalPose(const Eigen::Vector<double, 5>& goal_pose);
    void setInitialPose(const Eigen::Vector<double, 5>& initial_pose);
    void setObstacles(const std::vector<Obstacle>& obstacles);
    void runSQP(const SystemModel& system_model);
    void updateTrajectoryData();
    QPData setupQP(const SystemModel& system_model, Eigen::Matrix<double, 5, 5>& Q, Eigen::Matrix<double, 2, 2>& R);

    TrajectoryData getTrajectoryData() const {
        return {path_x_, path_y_, path_yaw_, velocity_, steering_angle_, acceleration_, steering_rate_};
    }
    inline const Eigen::VectorXd& getOptimalSolution() const { return optimal_solution_; }
    inline const std::vector<double>& getPathX() const { return path_x_; }
    inline const std::vector<double>& getPathY() const { return path_y_; }
    inline const std::vector<double>& getPathYaw() const { return path_yaw_; }

private:
    double trajectory_time_;
    double Ts_;
    int n_sqp_;
    int qp_iteration_;
    double rho_goal_, rho_obs_;
    Eigen::Vector<double, 5> state_lowerbound_, state_upperbound_;
    Eigen::Vector<double, 2> input_lowerbound_, input_upperbound_;
    std::vector<Obstacle> obstacles_;
    double safety_margin_;

    Eigen::VectorXd initial_guess_, optimal_solution_;

    Eigen::Vector<double, 5> x0_;
    Eigen::Vector<double, 5> x_goal_;
    Eigen::Vector<double, 2> u_goal_;

    Eigen::Matrix<double, 5, 5> Q_;
    Eigen::Matrix<double, 2, 2> R_;

    int prediction_horizon_;

    int state_dim_;
    int input_dim_;

    int nx_, nu_, total_vars_;
    int n_eq_;
    int n_ineq_;

    int n_slack_;
    int n_obstacle_constraints_;
    int n_obstacle_slack_;
    int total_vars_slack_;
    int total_vars_all_slack_;
    int total_constraints_;

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