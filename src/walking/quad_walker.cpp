#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "gait_planner.cpp"
#include "ik_solver.cpp"
#include "zmp_controller.cpp"
#include <algorithm>

using namespace std::chrono_literals;

class QuadControllerNode : public rclcpp::Node {
public:
  QuadControllerNode()
  : Node("quad_controller") {
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&QuadControllerNode::control_loop, this));
    this->gait_planner_.initializeTrot(1, 0.1, 0.07, -0.28);
  }

private:
  void control_loop() {
    using FootTargets = IKSolver::FootTargets;
    using JointAngles = IKSolver::JointAngles;
    FootTargets gait_targets = compute_foot_targets();
    JointAngles angles = ik_solver_.solve(gait_targets);
    JointAngles adjusted = zmp_ctrl_.adjust(angles);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();
    // Order: hip_front_left, knee_front_left, ... hip_back_right, knee_back_right
    for (const auto & leg : adjusted) {
      msg.data.push_back(leg[0]);
      msg.data.push_back(leg[1]);
    }

    pub_->publish(msg);
  }

  IKSolver::FootTargets compute_foot_targets() {
    double t = this->now().seconds();

    std::array<std::array<double, 4>, 4> trajectoryParameters = {{
      {{1.5, 0.05, 0.03, -0.29}}, // FL: step frequency in 1/s, step length in m, step height in m, ground in the hip coordinate system in m
      {{1.5, 0.05, 0.03, -0.29}}, // FR
      {{1.5, 0.05, 0.02, -0.29}}, // HL
      {{1.5, 0.05, 0.02, -0.29}}, // HR
    }};

    IKSolver::FootTargets targets; // targets order: FL, FR, HL, HR
    auto compute_target = [&](double phase, double step_length, double step_height, double z_ground) {
      IKSolver::FootTargets::value_type target;
      // Sinus-wave for forward movement
      target[0] = (step_length/2) * std::sin(phase);
      // No side movement possible
      target[1] = 0.0;
      // Clamp to 0 to smooth the non-swing phase
      target[2] = true ? step_height * std::clamp(std::sin(phase), 0.0, 1.0) + z_ground : z_ground+step_height/2;
      return target;
    };

    for (int i=0; i<4; ++i) {
      double freq = trajectoryParameters[i][0];
      double stepLength = trajectoryParameters[i][1];
      double stepHeight = trajectoryParameters[i][2];
      double z_ground = trajectoryParameters[i][3];

      double offset = 0.0;
      if (i==1 || i==2) offset = M_PI;
      double phase = std::fmod(2 * M_PI * freq * t + offset, 2 * M_PI);

      targets[i] = compute_target(phase, stepLength, stepHeight, z_ground);
    }

    return targets;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  GaitPlanner gait_planner_;
  IKSolver ik_solver_;
  ZMPController zmp_ctrl_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadControllerNode>());
  rclcpp::shutdown();
  return 0;
}