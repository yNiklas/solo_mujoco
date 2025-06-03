#ifndef SOLO_MUJOCO_MUJOCO_SIM_
#define SOLO_MUJOCO_MUJOCO_SIM_

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <map>
#include <string>
#include <mutex>
#include <random>
#include <vector>

namespace solo_mujoco {
class MuJoCoSimulator {
private:
    // This simulator is a singleton, so the system_interface plugin can easily use it to read states and write commands
    MuJoCoSimulator();

    void initializeFrameAndTargetBuffers();

    // Buffers the joint states (d->qpos & d->qvel) into the interface_name_to_state map
    void bufferStates();

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

    // Maps the joint name to the index for d->qpos and d->qvel
    std::map<std::string, int> joint_name_to_qpos_idx;
    std::map<std::string, int> joint_name_to_qvel_idx;

    // Maps the sensor name to the index for d->sensordata
    std::map<std::string, int> sensor_name_to_sensordata_idx;
    // Maps the sensor name to the noise specified in the XML file
    std::map<std::string, mjtNum> sensor_name_to_noise_stddev;
    // Maps the sensor name to the number of values emitted by them
    std::map<std::string, int> sensor_name_to_dim;
    // Buffer for sensor data to be exchanged with system_interface.
    // To be protected by state_buffer_mutex
    std::map<std::string, std::vector<mjtNum>> sensor_name_to_sensordata;

    // Buffers for the joint states, that will be given to ROS2-control
    // in the getJointState method. The method is called from another thread
    // so that we cannot directly access d->qpos and d->qvel.
    std::map<std::string, double> interface_name_to_state;
    std::mutex state_buffer_mutex;

    // Buffers for the targets (=commands) received from ROS2-control (maps interface_name -> target)
    std::map<std::string, double> interface_name_to_target;
    std::mutex target_buffer_mutex;

    // Buffers for the gains, which can be set once initially or updated
    std::map<std::string, double> joint_name_to_k_p; // Proportional gain per joint
    std::map<std::string, double> joint_name_to_k_d; // Derivative gain per joint
    std::map<std::string, double> joint_name_to_k_t; // Torque gain per joint
    std::mutex gain_buffer_mutex;

    double lastx = 0;
    double lasty = 0;

    // The static control callback is needed to set the mjcb_control global pointer
    static void staticControlCallback(const mjModel *m, mjData *d);
    void controlCallback(const mjModel *m, mjData *d);

    // Provides the joint states for ROS2-control system_interface plugin. Called from another thread!
    double getJointState(const std::string interface_name);

    // Provides the buffered sensor readings for ROS2-control system_interface plugin. Called from another thread!
    // The resulting vector contains all sensor outputs, as dimensioned in m->sensor_dim.
    // The values include noise.
    std::vector<mjtNum> getSensorValue(const std::string name);

    // Incoming ROS2-control targets (via system_interface), e.g., issued by external nodes via /position_controller/commands... topics.
    // The interface_name follows the form <joint_name>/<interface>, e.g., FL_HFE/position
    void acceptROS2ControlTarget(const std::string interface_name, const double value);

    // Set gains for the control loop. Set by ROS2-control
    void specifyKpGain(const std::string joint_name, const double gain);
    void specifyKdGain(const std::string joint_name, const double gain);
    void specifyKtGain(const std::string joint_name, const double gain); // Torque gain

    // Simulation itself, call from the system_interface in a separate thread
    static int startSimulation(const std::string &world_xml, const std::string &meshes_path);
    int simulate(const std::string &world_xml, const std::string &meshes_path);

    // GLFW window controls
    static void staticKeyboardCallback(GLFWwindow *window, int key, int scancode, int act, int mods);
    void keyboardCallback(GLFWwindow *window, int key, int scancode, int act, int mods);

    static void staticScrollCallback(GLFWwindow *window, double xoffset, double yoffset);
    void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);

    static void staticMouseButtonCallback(GLFWwindow *window, int button, int act, int mods);
    void mouseButtonCallback(GLFWwindow *window, int button, int act, int mods);

    static void staticMouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
    void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);

    static std::string getJointNameFromInterfaceName(const std::string& input) {
        size_t pos = input.find('/');
        if (pos != std::string::npos) {
            return input.substr(0, pos);
        }
        return input;
    }

    static std::string getInterfaceFromInterfaceName(const std::string& input) {
        size_t pos = input.find('/');
        if (pos != std::string::npos) {
            return input.substr(pos + 1);
        }
        return "";
    }
};
}

#endif