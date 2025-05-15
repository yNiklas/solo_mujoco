#ifndef SOLO_MUJOCO_SIMULATOR_
#define SOLO_MUJOCO_SIMULATOR_
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace solo_mujoco {
class Simulator : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::thread m_simulation; // Thread for MuJoCo simulation

    // MuJoCo parameters:
    std::string m_mujoco_model_xml_path;
    std::string m_meshes_path;
};
}
#endif
