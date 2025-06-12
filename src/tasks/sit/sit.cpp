#include "solo_mujoco/sit.hpp"
#include <algorithm>

namespace solo_mujoco {
Sit::Sit(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher) : jointPositionPublisher_(jointPositionPublisher) {}

void Sit::start(const double timestamp) {
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

std::string Sit::getName() const {
    return "SIT";
}
}