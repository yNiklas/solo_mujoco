#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ik_solver.cpp"
#include "zmp_controller.cpp"

using namespace std::chrono_literals;

class QuadControllerNode : public rclcpp::Node {
public:
  QuadControllerNode()
  : Node("quad_controller") {
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&QuadControllerNode::control_loop, this));
    // Initialisiere Ziele & Solver
    targets_ = {{{{0.2,0.0,-0.1}}, {{0.2,0.0,-0.1}}, {{-0.2,0.0,-0.1}}, {{-0.2,0.0,-0.1}}}};
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
    // Reihenfolge: hip_front_left, knee_front_left, ... hip_back_right, knee_back_right
    for (const auto & leg : adjusted) {
      msg.data.push_back(leg[0]);
      msg.data.push_back(leg[1]);
    }

    pub_->publish(msg);
  }

  IKSolver::FootTargets compute_foot_targets() {
    // Implementierung eines diagonalen Trott-Gaits
    // Phase-Offsets: FL/BR = 0, FR/BL = pi
    double t = this->now().seconds();
    double freq = 1;          // Schrittfrequenz Hz
    double step_length = 0.1;   // Schrittweite in Metern
    double step_height = 0.16;  // Hebehöhe in Metern

    // Zeitparametrisierung für Swing/Stance
    double phase_FL = std::fmod(2 * M_PI * freq * t + 0.0, 2 * M_PI);
    double phase_BR = phase_FL;
    double phase_FR = std::fmod(2 * M_PI * freq * t + M_PI, 2 * M_PI);
    double phase_BL = phase_FR;

    IKSolver::FootTargets targets;
    auto compute_target = [&](double phase) {
      IKSolver::FootTargets::value_type target;
      // Vorwärts-Rückwärts Bewegung: sinusförmig
      target[0] = (step_length/2) * std::sin(phase);
      // Seitenbewegung vernachlässigt
      target[1] = 0.0;
      // Hebung nur in Swing-Phase (phase < pi)
      target[2] = (phase < M_PI) ? step_height * std::sin(phase) : 0.0;
      return target;
    };

    targets[0] = compute_target(phase_FL); // front_left
    targets[1] = compute_target(phase_FR); // front_right
    targets[2] = compute_target(phase_BL); // back_left
    targets[3] = compute_target(phase_BR); // back_right

    return targets;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  IKSolver ik_solver_;
  ZMPController zmp_ctrl_;
  IKSolver::FootTargets targets_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadControllerNode>());
  rclcpp::shutdown();
  return 0;
}