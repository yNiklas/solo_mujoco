#include "solo_mujoco/system_interface.hpp"

#include <thread>
#include <vector>
#include <iostream>

#include "solo_mujoco/mujoco_simulator.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "HardwareInterface"

namespace solo_mujoco {
hardware_interface::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start simulation in parallel
    m_mujoco_model_xml_path = info_.hardware_parameters["mujoco_world_xml_path"];
    m_meshes_path = info_.hardware_parameters["meshes_path"];
    
    m_simulation = std::thread(MuJoCoSimulator::startSimulation, m_mujoco_model_xml_path, m_meshes_path);
    m_simulation.detach();

    if (info_.joints.size() != 8) {
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME),
                    "Expected 8 joints, got %ld",
                    info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME),
                    "Expected 3 command interfaces, got %ld",
                    joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!(joint.command_interfaces[0].name ==
                    hardware_interface::HW_IF_POSITION ||
                joint.command_interfaces[1].name ==
                    hardware_interface::HW_IF_VELOCITY ||
                joint.command_interfaces[2].name ==
                    hardware_interface::HW_IF_EFFORT)) {
            RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME),
                         "Joint '%s' needs the following command    interfaces in that "
                         "order: %s, %s, %s.",
                         joint.name.c_str(),    hardware_interface::HW_IF_POSITION,
                         hardware_interface::HW_IF_VELOCITY,    hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME),
                    "Expected 3 state interfaces, got %ld",
                    joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!(joint.state_interfaces[0].name ==
                  hardware_interface::HW_IF_POSITION ||
              joint.state_interfaces[0].name ==
                  hardware_interface::HW_IF_VELOCITY ||
              joint.state_interfaces[0].name ==        hardware_interface::HW_IF_EFFORT)) {
          RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                       "Joint '%s' needs the following state       interfaces in that "
                       "order: %s, %s, and %s.",
                       joint.name.c_str(),     hardware_interface::HW_IF_POSITION,
                       hardware_interface::HW_IF_VELOCITY,
                       hardware_interface::HW_IF_EFFORT);
          return hardware_interface::CallbackReturn::ERROR;
        }

        // Set gains in the simulator
        double p = std::stod(joint.parameters.at("p"));
        double d = std::stod(joint.parameters.at("d"));
        double t = std::stod(joint.parameters.at("t"));
        MuJoCoSimulator::getInstance().specifyKpGain(joint.name, p);
        MuJoCoSimulator::getInstance().specifyKdGain(joint.name, d);
        MuJoCoSimulator::getInstance().specifyKtGain(joint.name, t);
    }

    // Initialize IMU publisher
    node = rclcpp::Node::make_shared(NODE_NAME);
    std::chrono::milliseconds publishing_speed(1000 / 60);
    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_publish_timer = node->create_wall_timer(publishing_speed, std::bind(&Simulator::publishImuData, this));
    imu_publisher_thread = std::thread([this](){
        rclcpp::spin(node);
    });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Simulator::read(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    for (const auto &[interface_name, joint_desc] : joint_state_interfaces_) {
        set_state(interface_name, MuJoCoSimulator::getInstance().getJointState(interface_name));
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Simulator::write(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &duration) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    for (const auto &[interface_name, joint_desc] : joint_command_interfaces_) {
        double cmd = get_command(interface_name);
        if (!std::isnan(cmd)) {
            MuJoCoSimulator::getInstance().acceptROS2ControlTarget(interface_name, cmd);
        }
    }
    return hardware_interface::return_type::OK;
}

void Simulator::publishImuData() {
    auto now = node->get_clock()->now();
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = "imu_link";
    msg.header.stamp = node->get_clock()->now();

    std::vector<double> accelerometer = MuJoCoSimulator::getInstance().getSensorValue("accelerometer");
    std::vector<double> gyroscope = MuJoCoSimulator::getInstance().getSensorValue("gyroscope");
    std::vector<double> orientation = MuJoCoSimulator::getInstance().getSensorValue("orientation_sensor");
    if (accelerometer.size() == 3) {
        msg.linear_acceleration.x = accelerometer[0];
        msg.linear_acceleration.y = accelerometer[1];
        msg.linear_acceleration.z = accelerometer[2];
    }

    // todo: other

    imu_publisher->publish(msg);
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solo_mujoco::Simulator, hardware_interface::SystemInterface)
