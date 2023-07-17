cmake_minimum_required(VERSION 3.5)
project(xwr_data)

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
find_package(rclcpp_components REQUIRED)
find_package(serial REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ti_mmwave_msgs REQUIRED)

set(dependencies
  rclcpp
  pcl_ros
  pcl_conversions
  serial
  ti_mmwave_msgs
)

# Build

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
)

add_library(xwr_uart_reader_class SHARED src/mmWaveDataReader.cpp)

target_compile_definitions(xwr_uart_reader_class PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(xwr_uart_reader_class  ${dependencies} )
rclcpp_components_register_nodes(xwr_uart_reader_class "xwr_data::DataUARTHandler")

add_executable(xwr_uart_reader
  src/mmWaveDataReaderNode.cpp
)
target_link_libraries(xwr_uart_reader
  xwr_uart_reader_class
)

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
    xwr_uart_reader_class
    xwr_uart_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
