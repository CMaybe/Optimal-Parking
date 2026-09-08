#include <emscripten/bind.h>

#include "optimal_parking/system/system_model.hpp"
#include "optimal_parking/trajectory_optimizer.hpp"

using namespace emscripten;
using namespace optimal_parking;

namespace {

// Thin wrapper exposing only what the web frontend needs: build from a
// preloaded config file, feed poses in, run the planner, read the path out.
class Planner {
public:
    explicit Planner(const std::string& config_path) : optimizer_(config_path), model_(2.8, 1.6) {}

    void set_initial_pose(double x, double y, double yaw, double v, double steer) {
        optimizer_.set_initial_pose(Eigen::Vector<double, 5>(x, y, yaw, v, steer));
    }

    void set_goal_pose(double x, double y, double yaw, double v, double steer) {
        optimizer_.set_goal_pose(Eigen::Vector<double, 5>(x, y, yaw, v, steer));
    }

    void set_obstacles(const val& obstacles_js) {
        const unsigned length = obstacles_js["length"].as<unsigned>();
        std::vector<Obstacle> obstacles;
        obstacles.reserve(length);
        for (unsigned i = 0; i < length; ++i) {
            val item = obstacles_js[i];
            Obstacle obstacle;
            obstacle.center = Eigen::Vector2d(item["x"].as<double>(), item["y"].as<double>());
            obstacle.length = item["length"].as<double>();
            obstacle.width = item["width"].as<double>();
            obstacle.yaw = item["yaw"].as<double>();
            obstacles.push_back(obstacle);
        }
        optimizer_.set_obstacles(obstacles);
    }

    void set_safety_margin(double value) { optimizer_.set_safety_margin(value); }
    void set_goal_penalty(double value) { optimizer_.set_goal_penalty_weight(value); }
    void set_obstacle_penalty(double value) { optimizer_.set_obstacle_penalty_weight(value); }
    void set_sqp_iterations(int value) { optimizer_.set_max_sqp_iterations(value); }
    void set_qp_iterations(int value) { optimizer_.set_max_qp_iterations(value); }

    val plan() {
        optimizer_.run_sqp(model_);
        const TrajectoryData data = optimizer_.get_trajectory_data();

        val result = val::object();
        result.set("x", val::array(data.path_x));
        result.set("y", val::array(data.path_y));
        result.set("yaw", val::array(data.path_yaw));
        result.set("velocity", val::array(data.velocity));
        result.set("steeringAngle", val::array(data.steering_angle));
        result.set("acceleration", val::array(data.acceleration));
        result.set("steeringRate", val::array(data.steering_rate));
        return result;
    }

private:
    TrajectoryOptimizer optimizer_;
    SystemModel model_;
};

}  // namespace

EMSCRIPTEN_BINDINGS(optimal_parking) {
    class_<Planner>("Planner")
        .constructor<std::string>()
        .function("setInitialPose", &Planner::set_initial_pose)
        .function("setGoalPose", &Planner::set_goal_pose)
        .function("setObstacles", &Planner::set_obstacles)
        .function("setSafetyMargin", &Planner::set_safety_margin)
        .function("setGoalPenalty", &Planner::set_goal_penalty)
        .function("setObstaclePenalty", &Planner::set_obstacle_penalty)
        .function("setSqpIterations", &Planner::set_sqp_iterations)
        .function("setQpIterations", &Planner::set_qp_iterations)
        .function("plan", &Planner::plan);
}
