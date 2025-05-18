#define _USE_MATH_DEFINES

#include <optimal_parking/system/system_input.hpp>
#include <optimal_parking/system/system_model.hpp>
#include <optimal_parking/system/system_state.hpp>
#include <optimal_parking/trajectory_optimizer.hpp>
#include <optimal_parking/utils.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <matplotlibcpp.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace plt = matplotlibcpp;
using namespace optimal_parking;

void plot_car(const double& car_length,
              const double& car_width,
              const double& x,
              const double& y,
              const double& yaw,
              const double& steering_angle = 0.0,
              std::string line = "-k") {
    const double wheel_length = car_length / 3;
    const double wheel_width = car_width / 5;
    const double front_wheel_offset = car_length / 2;
    const double rear_wheel_offset = -car_length / 2;

    Eigen::Matrix2d rotate;
    rotate << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);

    Eigen::MatrixXd car(2, 5);
    car.row(0) << -car_length / 2, -car_length / 2, car_length / 2, car_length / 2, -car_length / 2;
    car.row(1) << car_width / 2, -car_width / 2, -car_width / 2, car_width / 2, car_width / 2;
    car = rotate * car;
    car.row(0).array() += x;
    car.row(1).array() += y;

    std::vector<double> car_x(car.cols());
    std::vector<double> car_y(car.cols());
    for (int i = 0; i < car.cols(); ++i) {
        car_x[i] = car(0, i);
        car_y[i] = car(1, i);
    }
    plt::plot(car_x, car_y, line);

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
    front_left_wheel.row(1).array() += car_width / 2;
    front_right_wheel.row(0).array() += front_wheel_offset;
    front_right_wheel.row(1).array() -= car_width / 2;

    Eigen::MatrixXd rear_left_wheel = wheel;
    Eigen::MatrixXd rear_right_wheel = wheel;

    rear_left_wheel.row(0).array() += rear_wheel_offset;
    rear_left_wheel.row(1).array() += car_width / 2;
    rear_right_wheel.row(0).array() += rear_wheel_offset;
    rear_right_wheel.row(1).array() -= car_width / 2;

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
    double car_length = config["car_length"].as<double>();
    double car_width = config["car_width"].as<double>();
    double Ts = config["Ts"].as<double>();
    int target_point_idx = 0;

    Eigen::Vector<double, 5> initial_pose =
        Eigen::Map<Eigen::Vector<double, 5>>(config["initial_pose"].as<std::vector<double>>().data());
    Eigen::Vector<double, 5> goal_pose =
        Eigen::Map<Eigen::Vector<double, 5>>(config["goal_pose"].as<std::vector<double>>().data());

    SystemModel system(car_length, car_width);
    TrajectoryOptimizer optimizer("../config.yaml");
    optimizer.setInitialPose(initial_pose);
    optimizer.setGoalPose(goal_pose);
    optimizer.runSQP(system);
    auto [traj_x, traj_y, traj_yaw, traj_acc, traj_steering_rate] = optimizer.getTrajectoryData();
    int len = traj_x.size();
    Eigen::Vector<double, 5> current_state = initial_pose;

    plt::figure_size(1440, 1080);
    std::map<std::string, std::string> options;
    options["color"] = "black";
    options["linestyle"] = "none";
    options["marker"] = "x";
    options["markersize"] = "10";

    for (int i = 0; i < len; i++) {
        plt::cla();
        plt::plot(traj_x, traj_y, options);

        plot_car(car_length, car_width, current_state[0], current_state[1], current_state[2], current_state[4]);
        plot_car(car_length, car_width, goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(4), "r-");
        plot_car(car_length, car_width, initial_pose(0), initial_pose(1), initial_pose(2), initial_pose(4), "g-");
        Eigen::Vector2d input(traj_acc[i], traj_steering_rate[i]);
        current_state = utils::RK4(system, current_state, input, Ts);
        plt::axis("equal");
        plt::xlim(-5, 20);
        plt::ylim(-5, 20);
        plt::grid(true);
        plt::pause(0.01);
    }
    plt::show();

    return 0;
}