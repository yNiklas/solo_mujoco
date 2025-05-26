#include "solo_mujoco/mujoco_simulator.hpp"

#include <filesystem>
#include <iostream>

namespace solo_mujoco {
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::staticControlCallback(const mjModel *m, mjData *d) {
    getInstance().controlCallback(m, d);
}

void MuJoCoSimulator::initializeFrameAndTargetBuffers() {
    // Set XML-defined initial qpos into simulation keyframe
    mju_copy(d->qpos, m->key_qpos, m->nq);

    for (int i=0; i<m->njnt; i++) {
        std::string name = m->names + m->name_jntadr[i];
        int qpos_idx = m->jnt_qposadr[i];

        // Set initial qpos as targets to prevent instant adjustment to (0, 0, ...) by the control loop before any targets from ROS2-control arrive
        interface_name_to_target[std::string(name + "/position")] = m->key_qpos[qpos_idx];
    }
}

// Control loop
void MuJoCoSimulator::controlCallback(
    [[maybe_unused]] const mjModel *m,
    [[maybe_unused]] mjData *d) {
    if (target_buffer_mutex.try_lock()) {
        for (int i=0; i < m->nu; ++i) {
            std::string joint_name = m->names + m->name_actuatoradr[i];

            double k_p = joint_name_to_k_p[joint_name];
            double k_d = joint_name_to_k_d[joint_name];
            double k_t = joint_name_to_k_t[joint_name];
            double currentPos = d->qpos[joint_name_to_qpos_idx[joint_name]];
            double currentVel = d->qvel[joint_name_to_qvel_idx[joint_name]];
            double targetPos = interface_name_to_target[std::string(joint_name + "/position")];
            double targetVel = interface_name_to_target[std::string(joint_name + "/velocity")];
            double targetTorque = interface_name_to_target[std::string(joint_name + "/effort")];

            // Debug output
            //std::cout << i << "/" << m->nu << " " << joint_name
            //    << ": pos[" << currentPos << " -> " << targetPos << "]"
            //    << " vel[" << currentVel << " -> " << targetVel << "]"
            //    << " eff[->" << targetTorque << "]"
            //    << " | Kp=" << k_p << ", Kd=" << k_d << ", Kt=" << k_t
            //    << std::endl;

            // Since the joints are <motor> actuators in mujoco/actuators.xml, d->ctrl[i] sets the torque of
            // the i'th joint
            d->ctrl[i] =
                k_p * (targetPos - currentPos) +
                k_d * (targetVel - currentVel) +
                k_t * targetTorque;
        }
        target_buffer_mutex.unlock();
    }
}

int MuJoCoSimulator::startSimulation(const std::string &world_xml, const std::string &meshes_path) {
    return getInstance().simulate(world_xml, meshes_path);
}

int MuJoCoSimulator::simulate(const std::string &world_xml, const std::string &meshes_path) {
    // Create filesystem
    auto vfs = std::make_unique<mjVFS>();
    mj_defaultVFS(vfs.get());

    std::string final_meshes_path = meshes_path.back() == '/' ? meshes_path : std::string(meshes_path + '/');
    // Iterate through meshes directory and add all meshes to the file system
    for (const auto &entry : std::filesystem::recursive_directory_iterator(meshes_path)) {
        if (std::filesystem::is_regular_file(entry.path())) {
            mj_addFileVFS(vfs.get(), final_meshes_path.c_str(), entry.path().filename().c_str());
        }
    }

    // Load model into fs
    char error_buffer[100] = "Model load failed";
    m = mj_loadXML(world_xml.c_str(), vfs.get(), error_buffer, 100);
    if (!m) {
        mju_error_s("Model load error: %s", error_buffer);
        return 1;
    }

    d = mj_makeData(m);
    initializeFrameAndTargetBuffers();

    // Init GLFW and create window
    if (!glfwInit()) {
        mju_error("Initialization of GLFW failed");
    }
    GLFWwindow *window = glfwCreateWindow(1200, 900, "Solo on Air", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Init own attributes
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Install callbacks for GLFW window
    glfwSetKeyCallback(window, staticKeyboardCallback);
    glfwSetScrollCallback(window, staticScrollCallback);

    // Connect our control callback to MuJoCo
    mjcb_control = MuJoCoSimulator::staticControlCallback;

    for (int i=0; i<m->njnt; i++) {
        std::string name = m->names + m->name_jntadr[i];
        int qpos_idx = m->jnt_qposadr[i];
        int qvel_idx = m->jnt_dofadr[i];
        joint_name_to_qpos_idx[name] = qpos_idx;
        joint_name_to_qvel_idx[name] = qvel_idx;
    }

    // Simulation loop
    while (!glfwWindowShouldClose(window)) {
        mjtNum closeLookupTime = d->time;
        while(d->time - closeLookupTime < 1.0/60.0) {
            mj_step(m, d);
            bufferStates();
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        
        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // Free buffers
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    mj_deleteVFS(vfs.get());
    
    return 0;
}

void MuJoCoSimulator::bufferStates() {
    if (state_buffer_mutex.try_lock()) {
        for (const auto &[joint_name, qpos_idx] : joint_name_to_qpos_idx) {
            std::string interface_name(joint_name + "/position");
            interface_name_to_state[interface_name] = d->qpos[qpos_idx];
        }
        for (const auto &[joint_name, qvel_idx] : joint_name_to_qvel_idx) {
            std::string interface_name(joint_name + "/velocity");
            interface_name_to_state[interface_name] = d->qvel[qvel_idx];
        }
        state_buffer_mutex.unlock();
    }
}

double MuJoCoSimulator::getJointState(const std::string interface_name) {
    return interface_name_to_state[interface_name];
}

void MuJoCoSimulator::acceptROS2ControlTarget(const std::string interface_name, const double value) {
    if (target_buffer_mutex.try_lock()) {
        interface_name_to_target[interface_name] = value;
        target_buffer_mutex.unlock();
    }
}

void MuJoCoSimulator::specifyKpGain(const std::string joint_name, const double gain) {
    if (gain_buffer_mutex.try_lock()) {
        joint_name_to_k_p[joint_name] = gain;
        gain_buffer_mutex.unlock();
    }
}
void MuJoCoSimulator::specifyKdGain(const std::string joint_name, const double gain) {
    if (gain_buffer_mutex.try_lock()) {
        joint_name_to_k_d[joint_name] = gain;
        gain_buffer_mutex.unlock();
    }
}
void MuJoCoSimulator::specifyKtGain(const std::string joint_name, const double gain) {
    if (gain_buffer_mutex.try_lock()) {
        joint_name_to_k_t[joint_name] = gain;
        gain_buffer_mutex.unlock();
    }
}

// GLFW window controls
void MuJoCoSimulator::staticKeyboardCallback(
    GLFWwindow *window,
    int key,
    int scancode,
    int act,
    int mods) {
    getInstance().keyboardCallback(window, key, scancode, act, mods);
}

void MuJoCoSimulator::keyboardCallback(
    [[maybe_unused]] GLFWwindow *window,
    [[maybe_unused]] int key,
    [[maybe_unused]] int scancode,
    [[maybe_unused]] int act,
    [[maybe_unused]] int mods) {
    // backspace-press resets the simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      initializeFrameAndTargetBuffers();
      mj_forward(m, d);
    }
}

void MuJoCoSimulator::staticScrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
    getInstance().scrollCallback(window, xoffset, yoffset);
}
void MuJoCoSimulator::scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
  (void)window;
  (void)xoffset;

  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
}
