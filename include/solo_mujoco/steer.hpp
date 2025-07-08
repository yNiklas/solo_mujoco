#ifndef STEER_
#define STEER_

#include "task.hpp"
#include "ik_solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

#include <iostream>

namespace solo_mujoco {
class Steer : public Task {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointPositionPublisher_;
    IKSolver ik_solver;

    double targetYawAngleInDeg; // in the world frame
    double currentYawAngleInDeg; // in the world frame

    IKSolver::FootTargets computeFootTargets(const double timestamp);

public:
    enum SteerDirection {
        LEFT,
        RIGHT
    };

    Steer(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> jointPositionPublisher);

    void execute(const double timestamp) override;
    void startReturnToStableStand(const double current_timestamp, const double desired_duration) override;
    void returnToStableStand(const double timestamp) override;
    bool returnedToStableStand(const double timestamp) override;
    std::string getName() const override;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;
        double w = msg->orientation.w;

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

        // yaw is 0 for the starting position (along x-axis)
        // yaw is PI and -PI for 180° (when robots looks in -x direction)
        // yaw is negative for the south direction (when robot looks in -y direction)
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        currentYawAngleInDeg = yaw * (180 / M_PI);
    }

    void steeringAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        targetYawAngleInDeg = msg->data;
    }

    bool reachedDestination() {
        return std::abs(targetYawAngleInDeg-currentYawAngleInDeg) < 5;
    }

    SteerDirection steerDirection() {
        if (targetYawAngleInDeg >= 0 && currentYawAngleInDeg >= 0) {
            return targetYawAngleInDeg < currentYawAngleInDeg ? LEFT : RIGHT;
        } else if (targetYawAngleInDeg < 0 && currentYawAngleInDeg < 0) {
            return currentYawAngleInDeg < targetYawAngleInDeg ? LEFT : RIGHT;
        } else {
            if (targetYawAngleInDeg >= 0) {
                if (targetYawAngleInDeg <= 90) {
                    return targetYawAngleInDeg > 180-std::abs(currentYawAngleInDeg) ? RIGHT : LEFT;
                } else {
                    return 180-targetYawAngleInDeg > std::abs(currentYawAngleInDeg) ? LEFT: RIGHT;
                }
            } else {
                if (currentYawAngleInDeg <= 90) {
                    return currentYawAngleInDeg > 180-std::abs(targetYawAngleInDeg) ? LEFT : RIGHT;
                } else {
                    return 180-currentYawAngleInDeg > std::abs(targetYawAngleInDeg) ? RIGHT: LEFT;
                }
            }
        }
    }
};
}

#endif
