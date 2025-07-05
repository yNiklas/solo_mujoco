#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <iostream>
#include "solo_mujoco/trot_walker.hpp"
#include "solo_mujoco/sit.hpp"
#include "solo_mujoco/steer.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class TaskControllerNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<Task> tasks[3];
    long unsigned int active_task_idx = 0;
    bool firstExecution = true;
    
    int next_task_idx = -1; // -1 = no task switch scheduled
    bool task_switch_initiated = false;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trot_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sit_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr steer_service_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    void initializeServices() {
        trot_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/tasks/trigger/TROT", 
            std::bind(&TaskControllerNode::handle_trot_trigger, this, std::placeholders::_1, std::placeholders::_2)
        );

        sit_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/tasks/trigger/SIT", 
            std::bind(&TaskControllerNode::handle_sit_trigger, this, std::placeholders::_1, std::placeholders::_2)
        );

        steer_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/tasks/trigger/STEER", 
            std::bind(&TaskControllerNode::handle_steer_trigger, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void handle_trot_trigger(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
        next_task_idx = 0;
        response->success = true;
        response->message = "TROT command triggered!";
        RCLCPP_INFO(this->get_logger(), "TROT service triggered");
    }

    void handle_sit_trigger(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
        next_task_idx = 1;
        response->success = true;
        response->message = "SIT command triggered!";
        RCLCPP_INFO(this->get_logger(), "SIT service triggered");
    }

    void handle_steer_trigger(
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
        next_task_idx = 2;
        response->success = true;
        response->message = "STEER command triggered!";

        RCLCPP_INFO(this->get_logger(), "STEER service triggered");
    }

    void controlLoop() {
        auto timestamp = this->now().seconds();

        // Detect desired task switch
        if (next_task_idx != -1) {
            if (!task_switch_initiated) {
                tasks[active_task_idx]->startReturnToStableStand(timestamp, 2);
                task_switch_initiated = true;
            }
            if (tasks[active_task_idx]->returnedToStableStand(timestamp)) {
                firstExecution = true;
                active_task_idx = next_task_idx;
                next_task_idx = -1;
                task_switch_initiated = false;
            }
        } else {
            if (firstExecution) {
                tasks[active_task_idx]->start(timestamp);
                firstExecution = false;
            }
            tasks[active_task_idx]->execute(timestamp);
        }
    }

public:
    TaskControllerNode() : Node("task_controller") {
        timer = this->create_wall_timer(20ms, std::bind(&TaskControllerNode::controlLoop, this));

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        tasks[0] = std::make_shared<solo_mujoco::TrotWalker>(joint_position_publisher);
        tasks[1] = std::make_shared<solo_mujoco::Sit>(joint_position_publisher);
        tasks[2] = std::make_shared<solo_mujoco::Steer>(joint_position_publisher);

        steering_angle_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/steering_angle",
            10,
            std::bind(&solo_mujoco::Steer::steeringAngleCallback, std::static_pointer_cast<solo_mujoco::Steer>(tasks[2]).get(), std::placeholders::_1)
        );

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            10,
            std::bind(&solo_mujoco::Steer::imuCallback, std::static_pointer_cast<solo_mujoco::Steer>(tasks[2]).get(), std::placeholders::_1)
        );

        initializeServices();
    }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskControllerNode>());
  rclcpp::shutdown();
  return 0;
}
