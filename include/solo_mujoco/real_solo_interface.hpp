#ifndef REAL_SOLO_INTERFACE_
#define REAL_SOLO_INTERFACE_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>
#include <mutex>
#include <thread>

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"

namespace solo_mujoco {
class RealSoloInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RealSoloInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Publisher of the sensor readings from the simulator
    std::shared_ptr<rclcpp::Node> node;
    std::thread publisher_thread;

    MasterBoardInterface* robot_if = nullptr;
    void initializeRobot(const std::string eth_interface);
    std::map<std::string, double> joint_name_to_k_p;
    std::map<std::string, double> joint_name_to_k_d;
    std::map<std::string, double> joint_name_to_k_t;
    std::map<std::string, int> joint_name_to_motor_index = {
        {"FL_HFE", 0},
        {"FL_KFE", 1},
        {"FR_HFE", 2},
        {"FR_KFE", 3},
        {"HL_HFE", 4},
        {"HL_KFE", 5},
        {"HR_HFE", 6},
        {"HR_KFE", 7}
    };
    std::map<std::string, double> interface_name_to_target;
    std::mutex interface_name_to_target_mutex;
    std::thread control_thread;
    void control(const int interval_in_milliseconds = 10);
    void controlCallback();

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher;
    rclcpp::TimerBase::SharedPtr imu_publish_timer;
    void publishImuData();

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> foot_contact_publisher;
    rclcpp::TimerBase::SharedPtr foot_contact_publish_timer;
    void publishFootContactData();
};
}

#endif
