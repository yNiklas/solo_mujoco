#include "solo_mujoco/sit.hpp"
#include <algorithm>

namespace solo_mujoco {
Sit::Sit(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher) : jointPositionPublisher_(jointPositionPublisher) {}

void Sit::start(const double timestamp) {
    startAngles = copyStableStandAngles();
    anglesTrajectory.push_back({{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.6, -0.7}},
        {{0.6, -0.7}}
    }});
    anglesTrajectory.push_back({{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.6, -0.7}},
        {{0.6, -1}}
    }});
    anglesTrajectory.push_back({{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.6, -1}},
        {{0.6, -1}}
    }});
    anglesTrajectory.push_back({{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.6, -1}},
        {{0.6, -1.5}}
    }});
    anglesTrajectory.push_back({{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.6, -1.5}},
        {{0.6, -1.5}}
    }});
    secondsPerPhase = {{0.2, 0.5, 0.5, 0.7, 0.7}};
    phaseStart = timestamp;
}

void Sit::execute([[maybe_unused]] const double timestamp) {
    if (phase >= secondsPerPhase.size()) {
        restInLastPosition();
        return;
    }

    // Check phase completion
    if (timestamp - phaseStart > secondsPerPhase[phase]) {
        phase++;
        if (phase == secondsPerPhase.size()) {
            // Trajectory done
            restInLastPosition();
            return;
        }
        std::copy(std::begin(anglesTrajectory[phase-1]), std::end(anglesTrajectory[phase-1]), std::begin(startAngles));
        phaseStart = timestamp;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();
    for (int leg=0; leg < 4; ++leg) {
        for (int joint=0; joint < 2; ++joint) {
            double start = startAngles[leg][joint];
            double dest = anglesTrajectory[phase][leg][joint];
            double direction = start <= dest ? 1 : -1;
            double distance = start <= dest ? dest-start : start-dest;
            double perc = (timestamp-phaseStart)/secondsPerPhase[phase];
            msg.data.push_back(start + direction * perc * distance);
        }
    }
    jointPositionPublisher_->publish(msg);
}

void Sit::restInLastPosition() {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();
    for (int leg=0; leg < 4; ++leg) {
        for (int joint=0; joint < 2; ++joint) {
            msg.data.push_back(anglesTrajectory[anglesTrajectory.size()-1][leg][joint]);
        }
    }
    jointPositionPublisher_->publish(msg);
}

void Sit::startReturnToStableStand(const double current_timestamp, [[maybe_unused]] const double desired_duration) {
    // Calculate time for returnal
    double returnal_duration = 0;
    for (long unsigned int i=0; i < phase; ++i) {
        returnal_duration += secondsPerPhase[i];
    }
    double in_step_time = 0;
    if (phase < secondsPerPhase.size()) in_step_time += (current_timestamp - phaseStart);
    returnal_duration += in_step_time;

    phase = secondsPerPhase.size() - phase;
    if (phase > 0) phase--;
    phaseStart = current_timestamp - in_step_time;

    std::reverse(anglesTrajectory.begin(), anglesTrajectory.end());
    if (phase == 0) {
        long unsigned int n = anglesTrajectory.size()-1;
        IKSolver::JointAngles copy;
        std::copy(std::begin(anglesTrajectory[n]), std::end(anglesTrajectory[n]), copy.begin());
    }

    stableStandReachedTimestamp = current_timestamp + returnal_duration + 0.05;
}

void Sit::returnToStableStand(const double timestamp) {
    execute(timestamp);
}

bool Sit::returnedToStableStand(const double timestamp) {
    return timestamp >= stableStandReachedTimestamp;
}

std::string Sit::getName() const {
    return "SIT";
}
}