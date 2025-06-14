#define _USE_MATH_DEFINES

#include <optimal_parking/system/system_input.hpp>
#include <optimal_parking/system/system_model.hpp>
#include <optimal_parking/system/system_state.hpp>
#include <optimal_parking/trajectory_optimizer.hpp>
#include <optimal_parking/types.hpp>
#include <optimal_parking/utils.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <matplotlibcpp.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace plt = matplotlibcpp;
using namespace optimal_parking;

void plot_obstacles(const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        Eigen::MatrixXd rect(2, 5);
        rect.row(0) << -obstacle.length / 2, -obstacle.length / 2, obstacle.length / 2, obstacle.length / 2, -obstacle.length / 2;
        rect.row(1) << -obstacle.width / 2, obstacle.width / 2, obstacle.width / 2, -obstacle.width / 2, -obstacle.width / 2;

        Eigen::Matrix2d rotate;
        rotate << std::cos(obstacle.yaw), -std::sin(obstacle.yaw), std::sin(obstacle.yaw), std::cos(obstacle.yaw);
        rect = rotate * rect;

        rect.row(0).array() += obstacle.center(0);
        rect.row(1).array() += obstacle.center(1);

        std::vector<double> rect_x(rect.cols());
        std::vector<double> rect_y(rect.cols());
        for (int i = 0; i < rect.cols(); ++i) {
            rect_x[i] = rect(0, i);
            rect_y[i] = rect(1, i);
        }

        plt::plot(rect_x, rect_y, "r-");
    }
}

void plot_vehicle(const double& vehicle_length,
                  const double& vehicle_width,
                  const double& x,
                  const double& y,
                  const double& yaw,
                  const double& steering_angle = 0.0,
                  std::string line = "-k") {
    const double wheel_length = vehicle_length / 3;
    const double wheel_width = vehicle_width / 5;
    const double front_wheel_offset = vehicle_length / 2;
    const double rear_wheel_offset = -vehicle_length / 2;

    Eigen::Matrix2d rotate;
    rotate << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

    Eigen::MatrixXd vehicle(2, 5);
    vehicle.row(0) << -vehicle_length / 2, -vehicle_length / 2, vehicle_length / 2, vehicle_length / 2, -vehicle_length / 2;
    vehicle.row(1) << vehicle_width / 2, -vehicle_width / 2, -vehicle_width / 2, vehicle_width / 2, vehicle_width / 2;
    vehicle = rotate * vehicle;
    vehicle.row(0).array() += x;
    vehicle.row(1).array() += y;

    std::vector<double> vehicle_x(vehicle.cols());
    std::vector<double> vehicle_y(vehicle.cols());
    for (int i = 0; i < vehicle.cols(); ++i) {
        vehicle_x[i] = vehicle(0, i);
        vehicle_y[i] = vehicle(1, i);
    }
    plt::plot(vehicle_x, vehicle_y, line);

    Eigen::MatrixXd wheel(2, 5);
    wheel.row(0) << -wheel_length / 2, -wheel_length / 2, wheel_length / 2, wheel_length / 2, -wheel_length / 2;
    wheel.row(1) << wheel_width / 2, -wheel_width / 2, -wheel_width / 2, wheel_width / 2, wheel_width / 2;

    Eigen::MatrixXd front_left_wheel = wheel;
    Eigen::MatrixXd front_right_wheel = wheel;

    Eigen::Matrix2d wheel_rotate;
    wheel_rotate << std::cos(steering_angle), -std::sin(steering_angle), std::sin(steering_angle), std::cos(steering_angle);
    front_left_wheel = wheel_rotate * front_left_wheel;
    front_right_wheel = wheel_rotate * front_right_wheel;

    front_left_wheel.row(0).array() += front_wheel_offset;
    front_left_wheel.row(1).array() += vehicle_width / 2;
    front_right_wheel.row(0).array() += front_wheel_offset;
    front_right_wheel.row(1).array() -= vehicle_width / 2;

    Eigen::MatrixXd rear_left_wheel = wheel;
    Eigen::MatrixXd rear_right_wheel = wheel;

    rear_left_wheel.row(0).array() += rear_wheel_offset;
    rear_left_wheel.row(1).array() += vehicle_width / 2;
    rear_right_wheel.row(0).array() += rear_wheel_offset;
    rear_right_wheel.row(1).array() -= vehicle_width / 2;

    front_left_wheel = rotate * front_left_wheel;
    front_right_wheel = rotate * front_right_wheel;
    rear_left_wheel = rotate * rear_left_wheel;
    rear_right_wheel = rotate * rear_right_wheel;

    front_left_wheel.row(0).array() += x;
    front_left_wheel.row(1).array() += y;
    front_right_wheel.row(0).array() += x;
    front_right_wheel.row(1).array() += y;
    rear_left_wheel.row(0).array() += x;
    rear_left_wheel.row(1).array() += y;
    rear_right_wheel.row(0).array() += x;
    rear_right_wheel.row(1).array() += y;

    std::vector<double> fl_x(front_left_wheel.cols());
    std::vector<double> fl_y(front_left_wheel.cols());
    std::vector<double> fr_x(front_right_wheel.cols());
    std::vector<double> fr_y(front_right_wheel.cols());
    for (int i = 0; i < front_left_wheel.cols(); ++i) {
        fl_x[i] = front_left_wheel(0, i);
        fl_y[i] = front_left_wheel(1, i);
        fr_x[i] = front_right_wheel(0, i);
        fr_y[i] = front_right_wheel(1, i);
    }
    plt::plot(fl_x, fl_y, "-b");
    plt::plot(fr_x, fr_y, "-b");

    // Plot rear wheels
    std::vector<double> rl_x(rear_left_wheel.cols());
    std::vector<double> rl_y(rear_left_wheel.cols());
    std::vector<double> rr_x(rear_right_wheel.cols());
    std::vector<double> rr_y(rear_right_wheel.cols());
    for (int i = 0; i < rear_left_wheel.cols(); ++i) {
        rl_x[i] = rear_left_wheel(0, i);
        rl_y[i] = rear_left_wheel(1, i);
        rr_x[i] = rear_right_wheel(0, i);
        rr_y[i] = rear_right_wheel(1, i);
    }
    plt::plot(rl_x, rl_y, "-b");
    plt::plot(rr_x, rr_y, "-b");
}

int main() {
    YAML::Node config = YAML::LoadFile("../config.yaml");
    double vehicle_length = config["vehicle_length"].as<double>();
    double vehicle_width = config["vehicle_width"].as<double>();
    double Ts = config["Ts"].as<double>();
    int target_point_idx = 0;
    std::vector<Obstacle> obstacles;
    Eigen::Vector<double, 5> initial_pose =
        Eigen::Map<Eigen::Vector<double, 5>>(config["initial_pose"].as<std::vector<double>>().data());
    Eigen::Vector<double, 5> goal_pose =
        Eigen::Map<Eigen::Vector<double, 5>>(config["goal_pose"].as<std::vector<double>>().data());

    SystemModel system(vehicle_length, vehicle_width);
    TrajectoryOptimizer optimizer("../config.yaml");
    optimizer.setInitialPose(initial_pose);
    optimizer.setGoalPose(goal_pose);
    optimizer.runSQP(system);
    auto [traj_x, traj_y, traj_yaw, _1, _2, traj_acc, traj_steering_rate] = optimizer.getTrajectoryData();
    int len = traj_x.size();
    Eigen::Vector<double, 5> current_state = initial_pose;
    for (const auto& obs : config["obstacles"]) {
        Obstacle obstacle;
        obstacle.center = Eigen::Map<Eigen::Vector2d>(obs["center"].as<std::vector<double>>().data());
        obstacle.length = obs["length"].as<double>();
        obstacle.width = obs["width"].as<double>();
        obstacle.yaw = obs["yaw"].as<double>();
        obstacles.push_back(obstacle);
    }

    plt::figure_size(1440, 1080);
    std::map<std::string, std::string> options;
    options["color"] = "black";
    options["linestyle"] = "none";
    options["marker"] = "x";
    options["markersize"] = "10";

    for (int i = 0; i < len; i++) {
        plt::cla();
        plt::plot(traj_x, traj_y, options);
        plot_obstacles(obstacles);

        plot_vehicle(vehicle_length, vehicle_width, current_state[0], current_state[1], current_state[2], current_state[4]);
        plot_vehicle(vehicle_length, vehicle_width, goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(4), "r-");
        plot_vehicle(vehicle_length, vehicle_width, initial_pose(0), initial_pose(1), initial_pose(2), initial_pose(4), "g-");
        Eigen::Vector2d input(traj_acc[i], traj_steering_rate[i]);
        current_state = utils::RK4(system, current_state, input, Ts);
        plt::axis("equal");
        plt::xlim(-20, 20);
        plt::ylim(-20, 20);
        plt::grid(true);
        plt::pause(0.01);
    }
    plt::show();

    return 0;
}