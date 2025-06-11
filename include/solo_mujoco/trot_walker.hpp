#ifndef TROT_WALKER_
#define TROT_WALKER_

#include "task.hpp"
#include "ik_solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace solo_mujoco {
class TrotWalker : public Task {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointPositionPublisher_;
    IKSolver ik_solver;

    IKSolver::FootTargets computeFootTargets(const double timestamp);

public:
    TrotWalker(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher);

    void execute(const double timestamp) override;
    std::string getName() const override;
};
}

#endif
