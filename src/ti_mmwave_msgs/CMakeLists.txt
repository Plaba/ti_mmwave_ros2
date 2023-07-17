cmake_minimum_required(VERSION 3.8)
project(ti_mmwave_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Declare ROS messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RadarScan.msg
  msg/DataFrame.msg
  msg/MMWaveConfig.msg
  srv/MMWaveCLI.srv
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
