cmake_minimum_required(VERSION 3.8)
project(solo_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_package(GLEW REQUIRED)

# MuJoCo
if(NOT DEFINED MUJOCO_DIR)
        set(MUJOCO_DIR "$ENV{HOME}/mujoco-3.3.2")
        message(WARNING "MUJOCO_DIR not specified. Using default ${MUJOCO_DIR}")
endif()
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES
        ${MUJOCO_DIR}/lib/libmujoco.so
)

# RPATH
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to use when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

# Build
add_library(${PROJECT_NAME} SHARED
  src/system_interface.cpp
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

#-
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

# X11 OpenGL: link with libmujoco210.so, libglew.so, libGL.so
target_link_libraries(mujoco_simulator PUBLIC
        ${MUJOCO_LIBRARIES}
        ${GLFW_LIBRARIES}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch meshes mujoco urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS mujoco_simulator
  DESTINATION lib
)

ament_package()
