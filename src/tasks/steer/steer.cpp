#include "solo_mujoco/steer.hpp"

namespace solo_mujoco {
Steer::Steer(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher) : jointPositionPublisher_(jointPositionPublisher) {}

IKSolver::FootTargets Steer::computeFootTargets(const double timestamp) {
    double t = timestamp;
    std::array<std::array<double, 4>, 4> trajectoryParameters;
    if (steerDirection() == Steer::SteerDirection::LEFT) {
        trajectoryParameters = {{
            {{1.5, 0.01, 0.03, -0.29}}, // FL: step frequency in 1/s,step length in m, step height in m, ground in the hip coordinate system in m
            {{1.5, 0.13, 0.03, -0.29}}, // FR
            {{1.5, 0.01, 0.02, -0.29}}, // HL
            {{1.5, 0.13, 0.02, -0.29}}, // HR
        }};
    } else {
        trajectoryParameters = {{
            {{1.5, 0.13, 0.03, -0.29}}, // FL: step frequency in 1/s,step length in m, step height in m, ground in the hip coordinate system in m
            {{1.5, 0.01, 0.03, -0.29}}, // FR
            {{1.5, 0.13, 0.02, -0.29}}, // HL
            {{1.5, 0.01, 0.02, -0.29}}, // HR
        }};
    }

    IKSolver::FootTargets targets; // targets order: FL, FR, HL, HR
    auto compute_target = [&](double phase, double step_length, double step_height, double z_ground) {
        IKSolver::FootTargets::value_type target;
        // Sinus-wave for forward movement
        target[0] = (step_length/2) * std::sin(phase);
        // No side movement possible
        target[1] = 0.0;
        // Clamp to 0 to smooth the non-swing phase
        target[2] = true ? step_height * std::clamp(std::sin(phase),  0.0, 1.0) + z_ground : z_ground+step_height/2;
        return target;
    };

    for (int i=0; i<4; ++i) {
        double freq = trajectoryParameters[i][0];
        double stepLength = trajectoryParameters[i][1];
        double stepHeight = trajectoryParameters[i][2];
        double z_ground = trajectoryParameters[i][3];
        double offset = 0.0;
        if (i==1 || i==2) offset = M_PI;
        double phase = std::fmod(2 * M_PI * freq * t + offset, 2 *M_PI);
        targets[i] = compute_target(phase, stepLength, stepHeight,z_ground);
    }
    return targets;
}

void Steer::execute(const double timestamp) {
    using FootTargets = IKSolver::FootTargets;
    using JointAngles = IKSolver::JointAngles;
    FootTargets gait_targets;
    JointAngles angles;
    if (reachedDestination()) {
        angles = copyStableStandAngles();
    } else {
        gait_targets = computeFootTargets(timestamp);
        angles = ik_solver.solve(gait_targets);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();

    // Order: hip_front_left, knee_front_left, ...hip_back_right,  knee_back_right
    for (const auto & leg : angles) {
        msg.data.push_back(leg[0]);
        msg.data.push_back(leg[1]);
    }
    jointPositionPublisher_->publish(msg);
}

void Steer::startReturnToStableStand([[maybe_unused]] const double current_timestamp, [[maybe_unused]] const double desired_duration) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();

    IKSolver::JointAngles stableStand = copyStableStandAngles();
    for (int leg=0; leg < 4; ++leg) {
      for (int joint=0; joint < 2; ++joint) {
        msg.data.push_back(stableStand[leg][joint]);
      }
    }
    
    jointPositionPublisher_->publish(msg);
}

void Steer::returnToStableStand([[maybe_unused]] const double timestamp) {}

bool Steer::returnedToStableStand([[maybe_unused]] const double timestamp) {
    // Since the joint angles are close to the stable stand position while walking,
    // resetting is done in one cycle. Hence, the stable stand is reached almost instantly
    return true;
}

std::string Steer::getName() const {
    return "STEER";
}
}
