#include "solo_mujoco/real_solo_interface.hpp"

#include <iostream>
#include <chrono>

#define NODE_NAME "RealSoloInterface"
#define N_SLAVES_CONTROLLED 4

namespace solo_mujoco {
hardware_interface::CallbackReturn RealSoloInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize Node for IMU and foot contact publisher
    node = rclcpp::Node::make_shared(NODE_NAME);
    std::chrono::milliseconds publishing_speed(1000 / 60);

    // Initialize IMU publisher and foot contact publisher
    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_publish_timer = node->create_wall_timer(publishing_speed, std::bind(&RealSoloInterface::publishImuData, this));
    foot_contact_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/foot_contacts", 10);
    foot_contact_publish_timer = node->create_wall_timer(publishing_speed, std::bind(&RealSoloInterface::publishFootContactData, this));

    publisher_thread = std::thread([this](){
        rclcpp::spin(node);
    });

    for (const auto &joint : info_.joints) {
        double p = std::stod(joint.parameters.at("p"));
        double d = std::stod(joint.parameters.at("d"));
        double t = std::stod(joint.parameters.at("t"));
        joint_name_to_k_p[joint.name] = p;
        joint_name_to_k_d[joint.name] = d;
        joint_name_to_k_t[joint.name] = t;
    }
    std::string eth_interface_name = info_.hardware_parameters["eth_interface"];
    initializeRobot(eth_interface_name);

    control_thread = std::thread(&RealSoloInterface::control, this, 10);
    control_thread.detach();

    return hardware_interface::CallbackReturn::SUCCESS;
}

void RealSoloInterface::initializeRobot(const std::string eth_interface) {
    // Initialize the robot with the given Ethernet interface
    robot_if = new MasterBoardInterface(eth_interface, false);
    if (!robot_if->Init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize robot on interface %s", eth_interface.c_str());
    } else {
        RCLCPP_INFO(node->get_logger(), "Robot initialized successfully on interface %s", eth_interface.c_str());
    }

    for (int i = 0; i < N_SLAVES_CONTROLLED; ++i) {
        robot_if->motor_drivers[i].motor1->SetCurrentReference(0);
        robot_if->motor_drivers[i].motor2->SetCurrentReference(0);
        robot_if->motor_drivers[i].motor1->Enable();
        robot_if->motor_drivers[i].motor2->Enable();
        robot_if->motor_drivers[i].EnablePositionRolloverError();
        robot_if->motor_drivers[i].SetTimeout(5);
        robot_if->motor_drivers[i].Enable();
    }

    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
	while (!robot_if->IsTimeout() && !robot_if->IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001) {
			last = std::chrono::system_clock::now();
			robot_if->SendInit();
		}
	}

    if (robot_if->IsTimeout()) {
        RCLCPP_ERROR(node->get_logger(), "Timeout while waiting for robot initialization.");
        return;
    }

    // Give the motors some time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    for (int i = 0; i < N_SLAVES_CONTROLLED * 2; ++i) {
        if (robot_if->motor_drivers[i / 2].IsConnected()) {
            RCLCPP_INFO(node->get_logger(), "Motor driver board %d connected successfully.", i);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Motor driver board %d is not connected.", i);
            continue;
        }

        if (robot_if->motors[i].IsEnabled() && robot_if->motors[i].IsReady()) {
            RCLCPP_INFO(node->get_logger(), "Motor %d is enabled.", i);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Motor %d is not enabled.", i);
        }
    }
}

void RealSoloInterface::control(const int interval_in_milliseconds) {
    while (rclcpp::ok()) {
        controlCallback();
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_in_milliseconds));
    }
    RCLCPP_INFO(node->get_logger(), "Control loop stopped.");
}

void RealSoloInterface::controlCallback() {
    std::map<std::string, double> local_if_to_target;
    if (interface_name_to_target_mutex.try_lock()) {
        for (const auto &[interface_name, target_value] : interface_name_to_target) {
            local_if_to_target[interface_name] = target_value;
        }
        interface_name_to_target_mutex.unlock();

        for (const auto &[joint_name, motor_index] : joint_name_to_motor_index) {
            if (robot_if->IsTimeout()) {
                RCLCPP_ERROR(node->get_logger(), "Robot interface timeout while controlling joint %s.", joint_name.c_str());
                continue;
            }
            if (!robot_if->motor_drivers[motor_index / 2].IsConnected()) {
                RCLCPP_ERROR(node->get_logger(), "Motor driver board for joint %s is not connected.", joint_name.c_str());
                continue;
            }
            if (robot_if->motor_drivers[motor_index / 2].error_code == 0xf) {
                RCLCPP_ERROR(node->get_logger(), "Transaction with %s failed.", joint_name.c_str());
                continue;
            }

            double target_position = local_if_to_target[joint_name + "/" + hardware_interface::HW_IF_POSITION];
            double target_velocity = local_if_to_target[joint_name + "/" + hardware_interface::HW_IF_VELOCITY];
            double position_error = target_position - get_state(joint_name + "/" + hardware_interface::HW_IF_POSITION);
            double velocity_error = target_velocity - get_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
            double target_torque = joint_name_to_k_p[joint_name] * position_error + joint_name_to_k_d[joint_name] * velocity_error;
            double applied_current = target_torque / 0.225;
            if (std::abs(applied_current) > 12) {
                applied_current = std::copysign(1.0, applied_current);
            }
            robot_if->motors[motor_index].SetCurrentReference(applied_current);
        }
    }
}

hardware_interface::return_type RealSoloInterface::read(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    for (const auto &[joint_name, motor_index] : joint_name_to_motor_index) {
        if (robot_if->IsTimeout()) {
            RCLCPP_ERROR(node->get_logger(), "Robot interface timeout while reading joint state.");
            return hardware_interface::return_type::ERROR;
        }
        if (!robot_if->motor_drivers[motor_index / 2].IsConnected()) {
            RCLCPP_ERROR(node->get_logger(), "Motor driver board for joint %s is not connected.", joint_name.c_str());
            continue;
        }
        if (robot_if->motor_drivers[motor_index / 2].error_code == 0xf) {
            RCLCPP_ERROR(node->get_logger(), "Transaction with %s failed.", joint_name.c_str());
            continue;
        }

        double position = robot_if->motors[motor_index].GetPosition();
        double velocity = robot_if->motors[motor_index].GetVelocity();

        set_state(joint_name + "/" + hardware_interface::HW_IF_POSITION, position);
        set_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealSoloInterface::write(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &duration) {
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    if (interface_name_to_target_mutex.try_lock()) {
        for (const auto &[interface_name, joint_desc] : joint_command_interfaces_) {
            double cmd = get_command(interface_name);
            if (!std::isnan(cmd)) {
                //std::cout << "Writing command for " << interface_name << ": " << cmd << std::endl;
                interface_name_to_target[interface_name] = cmd;
            }
        }
        interface_name_to_target_mutex.unlock();
    }
    
    return hardware_interface::return_type::OK;
}

void RealSoloInterface::publishImuData() {
    robot_if->ParseSensorData();
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = node->get_clock()->now();

    imu_msg.linear_acceleration.x = robot_if->imu_data.accelerometer[0];
    imu_msg.linear_acceleration.y = robot_if->imu_data.accelerometer[1];
    imu_msg.linear_acceleration.z = robot_if->imu_data.accelerometer[2];

    imu_msg.angular_velocity.x = robot_if->imu_data.gyroscope[0];
    imu_msg.angular_velocity.y = robot_if->imu_data.gyroscope[1];
    imu_msg.angular_velocity.z = robot_if->imu_data.gyroscope[2];

    // todo: set orientation from IMU quaternion

    imu_publisher->publish(imu_msg);
}

void RealSoloInterface::publishFootContactData() {
    // todo: Currently not implemented in the SDK?
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solo_mujoco::RealSoloInterface, hardware_interface::SystemInterface)

