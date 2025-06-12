#ifndef SIT_
#define SIT_

#include "task.hpp"
#include "ik_solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace solo_mujoco {
class Sit : public Task {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointPositionPublisher_;
    IKSolver ik_solver;

    IKSolver::JointAngles startAngles = {{
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.4, -0.7}},
        {{0.4, -0.7}}
    }};
    std::vector<IKSolver::JointAngles> anglesTrajectory;
    std::vector<double> secondsPerPhase;
    long unsigned int phase = 0;
    double phaseStart = 0; // timestamp in seconds

    void restInLastPosition();

public:
    Sit(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher);

    void start(const double timestamp) override;
    void execute(const double timestamp) override;
    std::string getName() const override;
};
}

#endif
