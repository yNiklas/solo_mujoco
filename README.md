# The First Steps of Solo - Sim2Real for a Quadruped Robot
This repository provides a ROS2 package including a MuJoCo simulation environment, tasks and an interface to the physical hardware of the Solo8 quadruped robot by [Open Dynamic Robot Initiative](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_8dof_v2/README.md#quadruped-robot-8dof-v2).

To build and source the package run:
```
colcon build --packages-select solo_mujoco
source install/setup.bash
```

## Simulation Environment
The simulation environment is powered by the [MuJoCo physics engine](https://mujoco.org/) and connected to ROS2-control via a [system interface](src/system_interface.cpp). This system interface accepts control targets from ROS2 topics and publishes state information.

Start the simulation with
```
ros2 launch solo_mujoco simulate.launch.py
```
Options:
|Option|Type|Description|
|---|---|---|
|plot|bool|Whether to start rqt alongside with the simulation|

### Sending Commands to the Simulation
The simulation accepts commands to the following topics:
|Topic|Description|
|---|---|
|/position_controller/commands|Accepts joint position targets as `Float64MultiArray` in the order hip_front_left, knee_front_left, ..., hip_back_right,  knee_back_right|
|/velocity_controller/commands|Accepts joint velocity targets as `Float64MultiArray` in the order hip_front_left, knee_front_left, ..., hip_back_right,  knee_back_right

### State Published by the Simulation
The system interface relays the following data to the ROS2 system:
|Topic|Frequency|Description|
|---|---|---|
|/imu|60Hz|Publishes an `Imu` object containing acceleration, angular velocity and quaternion orientation|
|/foot_contacts|Publishes a `Float64MultiArray` of length 4 with the vertical force. Order: [FL, FR, HL, HR]|


## Real Robot Interface
The [real robot system interface](src/real_solo_interface.cpp) connects to the real robot using the [Open Dynamic Robot Initiative SDK](https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk). The built SDK files are places in the [master_biard_sdk_libs](master_board_sdk_libs/) folder.

It accepts the same control targets as the simulation.

Launch the real robot interface with:
```
ros2 launch solo_mujoco real_solo.launch.py
```
Options:
|Option|Type|Description|
|---|---|---|
|eth_interface|string|The ethernet interface name of the robot|

## Task Controller
The repository comes along with a [task controller](src/tasks/task_controller.cpp) which manages the execution of several tasks in the simulation and on the real robot, e.g., walking or sitting.

To change the executed task, use the following services. All services accept a `std_srvs/srv/Trigger` message:
|Service|Description|
|---|---|
|/tasks/trigger/TROT|Triggers a walk of the robot with a trot pattern|
|/tasks/trigger/SIT|Triggers the robot to sit|
|/tasks/trigger/STEER|The robot walks a curve to reach a specified steering angle and then maintains a stable stand. The steering angle can be specified by publishing the angle in degrees as `std_msgs/msg/Float64` to the `/steering_angle` topic. For the simulation holds: East=0°, South=-90°, North=90°


## Installation
1. Make sure to place a MuJoCo release somewhere in the ROS2 workspace and reference it with the `MUJOCO_DIR` environment variable.
2. Make sure to have the `rclcpp`, `pluginlib`, `hardware_interface`, `glfw3` and `GLEW` packages installed. See the [CMakeLists.txt](CMakeLists.txt) file for details.
