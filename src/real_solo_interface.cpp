#include "solo_mujoco/real_solo_interface.hpp"

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"

#include <iostream>

#define NODE_NAME "RealSoloInterface"

namespace solo_mujoco {
hardware_interface::CallbackReturn RealSoloInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize Node for IMU and foot contact publisher
    node = rclcpp::Node::make_shared(NODE_NAME);
    std::chrono::milliseconds publishing_speed(1000 / 60);

    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_publish_timer = node->create_wall_timer(publishing_speed, std::bind(&RealSoloInterface::publishImuData, this));

    foot_contact_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/foot_contacts", 10);
    foot_contact_publish_timer = node->create_wall_timer(publishing_speed, std::bind(&RealSoloInterface::publishFootContactData, this));

    publisher_thread = std::thread([this](){
        rclcpp::spin(node);
    });

    //MasterBoardInterface robot_if("AAAA");
    //robot_if.Init();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealSoloInterface::read(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealSoloInterface::write(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &duration) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    for (const auto &[interface_name, joint_desc] : joint_command_interfaces_) {
        double cmd = get_command(interface_name);
        if (!std::isnan(cmd)) {
            std::cout << "Writing command for " << interface_name << ": " << cmd << std::endl;
        }
    }
    return hardware_interface::return_type::OK;
}

void RealSoloInterface::publishImuData() {
    
}

void RealSoloInterface::publishFootContactData() {
    
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solo_mujoco::RealSoloInterface, hardware_interface::SystemInterface)

