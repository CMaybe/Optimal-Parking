#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <vector>

#include "optimal_parking/system/system_input.hpp"
#include "optimal_parking/system/system_model.hpp"
#include "optimal_parking/system/system_state.hpp"

namespace optimal_parking {

struct QPData {
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
};

struct TrajectoryData {
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;

    std::vector<double> acceleration;
    std::vector<double> steering_rate;
};

class TrajectoryOptimizer {
public:
    TrajectoryOptimizer(const std::string& config_path);
    TrajectoryOptimizer(const double& trajectory_time,
                        const double& Ts,
                        const int& n_sqp,
                        const int& max_iteration,
                        const double& rho_slack,
                        const Eigen::Vector<double, 5>& state_weight,
                        const Eigen::Vector<double, 2>& input_weight,
                        const Eigen::Vector<double, 5>& state_lowerbound,
                        const Eigen::Vector<double, 5>& state_upperbound,
                        const Eigen::Vector<double, 2>& input_lowerbound,
                        const Eigen::Vector<double, 2>& input_upperbound);
    void setGoalPose(const Eigen::Vector<double, 5>& goal_pose);
    void setInitialPose(const Eigen::Vector<double, 5>& initial_pose);

    void runSQP(const SystemModel& system_model);
    QPData setupQP(const SystemModel& system_model, Eigen::Matrix<double, 5, 5>& Q, Eigen::Matrix<double, 2, 2>& R);
    void updateTrajectoryData();

    TrajectoryData getTrajectoryData() const { return {path_x_, path_y_, path_yaw_, acceleration_, steering_rate_}; }
    inline const Eigen::VectorXd& getOptimalSolution() const { return optimal_solution_; }
    inline const std::vector<double>& getPathX() const { return path_x_; }
    inline const std::vector<double>& getPathY() const { return path_y_; }
    inline const std::vector<double>& getPathYaw() const { return path_yaw_; }

private:
    double trajectory_time_;
    double Ts_;
    int n_sqp_;
    int max_iteration_;
    double rho_slack_;
    Eigen::Vector<double, 5> state_lowerbound_, state_upperbound_;
    Eigen::Vector<double, 2> input_lowerbound_, input_upperbound_;

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
    int total_vars_slack_;

    std::vector<double> path_x_;
    std::vector<double> path_y_;
    std::vector<double> path_yaw_;
    std::vector<double> acceleration_;
    std::vector<double> steering_rate_;
};
}  // namespace optimal_parking
#endif  // TRAJECTORY_OPTIMIZER_HPP