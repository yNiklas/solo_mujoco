#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include "solo_mujoco/trot_walker.hpp"
#include "solo_mujoco/sit.hpp"

using namespace std::chrono_literals;

class TaskControllerNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<Task> tasks[2];
    bool firstExecution = true;

    void controlLoop() {
        if (firstExecution) {
            tasks[0]->start(this->now().seconds());
            firstExecution = false;
        }
        tasks[0]->execute(this->now().seconds());
    }
public:
    TaskControllerNode() : Node("task_controller") {
        timer = this->create_wall_timer(20ms, std::bind(&TaskControllerNode::controlLoop, this));

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        tasks[1] = std::make_unique<solo_mujoco::TrotWalker>(joint_position_publisher);
        tasks[0] = std::make_unique<solo_mujoco::Sit>(joint_position_publisher);
    }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskControllerNode>());
  rclcpp::shutdown();
  return 0;
}
