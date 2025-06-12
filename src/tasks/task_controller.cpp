#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <iostream>
#include "solo_mujoco/trot_walker.hpp"
#include "solo_mujoco/sit.hpp"

using namespace std::chrono_literals;

class TaskControllerNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<Task> tasks[2];
    long unsigned int active_task_idx = 0;
    bool firstExecution = true;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trot_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sit_service_;

    void initializeServices() {

    }

    void handle_trot_trigger(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        response->message = "TROT command triggered!";
        
        RCLCPP_INFO(this->get_logger(), "TROT service triggered");
    }

    void handle_sit_trigger(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        response->message = "SIT command triggered!";
        
        RCLCPP_INFO(this->get_logger(), "SIT service triggered");
    }

    void controlLoop() {
        if (firstExecution) {
            tasks[active_task_idx]->start(this->now().seconds());
            firstExecution = false;
        }
        tasks[active_task_idx]->execute(this->now().seconds());
    }

public:
    TaskControllerNode() : Node("task_controller") {
        timer = this->create_wall_timer(20ms, std::bind(&TaskControllerNode::controlLoop, this));

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        tasks[0] = std::make_unique<solo_mujoco::TrotWalker>(joint_position_publisher);
        tasks[1] = std::make_unique<solo_mujoco::Sit>(joint_position_publisher);
    }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskControllerNode>());
  rclcpp::shutdown();
  return 0;
}
