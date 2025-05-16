#ifndef SOLO_MUJOCO_MUJOCO_SIM_
#define SOLO_MUJOCO_MUJOCO_SIM_

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <map>
#include <string>
#include <mutex>

namespace solo_mujoco {
class MuJuCoSimulator {
private:
    // This simulator is a singleton, so the system_interface plugin can easily use it to read states and write commands
    MuJuCoSimulator();

public:
    MuJoCoSimulator(const MuJoCoSimulator &) = delete;
    MuJoCoSimulator &operator=(const MuJoCoSimulator &) = delete;
    MuJoCoSimulator(MuJoCoSimulator &&) = delete;
    MuJoCoSimulator &operator=(MuJoCoSimulator &&) = delete;

    static MuJoCoSimulator &getInstance() {
      static MuJoCoSimulator simulator;
      return simulator;
    }

    // MuJoCo data structures
    mjModel *m = NULL;  // MuJoCo model
    mjData *d = NULL;   // MuJoCo data
    mjvCamera cam;      // abstract camera
    mjvOption opt;      // visualization options
    mjvScene scn;       // abstract scene
    mjrContext con;     // custom GPU context

    // Buffers for the targets (=commands) received from ROS2-control
    std::map<std::string, double> pos_targets;
    std::map<std::string, double> vel_targets;
    std::map<std::string, double> eff_targets;

    std::mutex cmd_buffer_mutex;

    // The static control callback is needed to set the mjcb_control global pointer
    static void staticControlCallback(const mjModel *m, mjData *d);
    void controlCallback(const mjModel *m, mjData *d);

    // Provides the joint states for ROS2-control system_interface plugin
    double getJointState(std::string joint_name);

    // Incoming ROS2-control targets (via system_interface), e.g., issued by external nodes via /position_controller/commands... topics
    void acceptROS2ControlTargetPosition(std::string joint_name, double targetPosition);
    void acceptROS2ControlTargetVelocity(std::string joint_name, double targetVelocity);
    void acceptROS2ControlTargetEffort(std::string joint_name, double targetEffort);
}
}

#endif