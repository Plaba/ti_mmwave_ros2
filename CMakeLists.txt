cmake_minimum_required(VERSION 3.5)
project(ti_mmwave_ros2)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_compile_options(-Wall -Werror)

# Find packages and libraries
find_package(Boost)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  PCL
  pcl_ros
  pcl_conversions
  builtin_interfaces
  serial
)

# Build

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(node_plugins "")

# Declare ROS messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/MMWaveCLI.srv
  msg/RadarScan.msg
  DEPENDENCIES builtin_interfaces std_msgs
)

# mmwave library

add_library(mmwave_comm_srv
  src/mmWaveCommSrv.cpp
)
target_compile_definitions(mmwave_comm_srv
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(mmwave_comm_srv
  ${dependencies}
)
rclcpp_components_register_nodes(mmwave_comm_srv "ti_mmwave_ros2::mmWaveCommSrv")
rosidl_target_interfaces(mmwave_comm_srv ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(data_handler_class
  src/DataHandlerClass.cpp
)
target_compile_definitions(data_handler_class
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(data_handler_class
  ${dependencies}
)
rclcpp_components_register_nodes(data_handler_class "ti_mmwave_ros2::DataUARTHandler")
rosidl_target_interfaces(data_handler_class ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executable: ti_mmwave_waveloader
add_executable(ti_mmwave_waveloader
  src/mmWaveLoader.cpp
  include/ti_mmwave_ros2/point_types.h
)
ament_target_dependencies(ti_mmwave_waveloader
  ${dependencies}
)

rosidl_target_interfaces(ti_mmwave_waveloader ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(ti_mmwave_waveloader
  data_handler_class
  mmwave_comm_srv
)

# Executable: mmWaveQuickConfig
add_executable(mmWaveQuickConfig
  src/mmWaveQuickConfig.cpp
  src/ParameterSetter.cpp
)
ament_target_dependencies(mmWaveQuickConfig
  ${dependencies}
)

rosidl_target_interfaces(mmWaveQuickConfig ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Install
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    mmwave_comm_srv
    data_handler_class
    ti_mmwave_waveloader
    mmWaveQuickConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

