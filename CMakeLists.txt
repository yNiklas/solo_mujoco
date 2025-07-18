cmake_minimum_required(VERSION 3.8)
project(solo_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#------------------------------------------
# Dependencies
#------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(control_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(GLEW REQUIRED)

#------------------------------------------
# Message Generation
#------------------------------------------
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Footstep.msg"
#  "msg/FootstepArray.msg"
#  DEPENDENCIES geometry_msgs builtin_interfaces
#)

#------------------------------------------
# MuJoCo
#------------------------------------------
if(NOT DEFINED MUJOCO_DIR)
  set(MUJOCO_DIR "$ENV{HOME}/mujoco-3.3.2")
  message(WARNING "MUJOCO_DIR not specified. Using default ${MUJOCO_DIR}")
endif()
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES
  ${MUJOCO_DIR}/lib/libmujoco.so
)

#------------------------------------------
# Master Board SDK
#------------------------------------------
set(MASTER_BOARD_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/master_board_sdk)
set(MASTER_BOARD_SDK_OBJECT_FILES
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/ESPNOW_manager.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/ESPNOW_types.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/ETHERNET_types.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/Link_manager.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/master_board_interface.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/motor_driver.o
  ${PROJECT_SOURCE_DIR}/master_board_sdk_libs/motor.o
)

#------------------------------------------
# RPATH
#------------------------------------------
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

#------------------------------------------
# Build Targets
#------------------------------------------

# System Interface Library
add_library(${PROJECT_NAME} SHARED
  src/system_interface.cpp
  src/tasks/walking/trot_walker.cpp
  src/tasks/walking/ik_solver.cpp
  src/tasks/sit/sit.cpp
  src/tasks/steer/steer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)
target_link_libraries(${PROJECT_NAME}
  mujoco_simulator
)

pluginlib_export_plugin_description_file(hardware_interface solo_mujoco_simulator.xml)

# MuJoCo Simulation Library
add_library(mujoco_simulator SHARED
  src/mujoco_simulator.cpp
)
target_include_directories(mujoco_simulator PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)
ament_target_dependencies(mujoco_simulator PUBLIC
  rclcpp
  sensor_msgs
)
target_link_libraries(mujoco_simulator PUBLIC
  ${MUJOCO_LIBRARIES}
  ${GLFW_LIBRARIES}
)

# Real Solo Interface Library
add_library(real_solo_interface SHARED
  src/real_solo_interface.cpp
)
target_include_directories(real_solo_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MASTER_BOARD_SDK_INCLUDE_DIR}
)
ament_target_dependencies(real_solo_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
)
target_link_libraries(real_solo_interface PUBLIC
  ${MASTER_BOARD_SDK_OBJECT_FILES}
)

pluginlib_export_plugin_description_file(hardware_interface real_solo_interface.xml)

# Walking Planner Node
#add_executable(footstep_planner_node
#  src/walking/footstep_planner_node.cpp
#)
#ament_target_dependencies(footstep_planner_node
#  rclcpp
#  geometry_msgs
#  builtin_interfaces
#  rosidl_default_runtime
#)
#target_include_directories(footstep_planner_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/#rosidl_generator_cpp>
#)
#target_link_libraries(footstep_planner_node
#  ${PROJECT_NAME}__rosidl_typesupport_cpp
#)

# Task controller node
add_executable(task_controller src/tasks/task_controller.cpp)
ament_target_dependencies(
  task_controller
  rclcpp
  std_srvs
)
target_link_libraries(task_controller ${PROJECT_NAME})

#------------------------------------------
# Install
#------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch meshes mujoco urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mujoco_simulator
  DESTINATION lib
)

install(TARGETS
  real_solo_interface
  DESTINATION lib
)

#install(TARGETS
#  footstep_planner_node
#  DESTINATION lib/${PROJECT_NAME}
#)

install(
  TARGETS task_controller
  DESTINATION lib/${PROJECT_NAME}
)

#ament_export_dependencies(rosidl_default_runtime)

ament_package()
