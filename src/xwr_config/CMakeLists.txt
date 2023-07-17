cmake_minimum_required(VERSION 3.8)
project(xwr_config)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial REQUIRED)
find_package(ti_mmwave_msgs REQUIRED)

set (dependencies
  rclcpp
  serial
  ti_mmwave_msgs
  rclcpp_components
)

include_directories(
  include
)

# xwr_config nodelet
add_library(xwr_config_srv_class SHARED
  src/mmWaveCommSrv.cpp
)
target_compile_definitions(xwr_config_srv_class
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(xwr_config_srv_class
  ${dependencies}
)
rclcpp_components_register_nodes(xwr_config_srv_class "xwr_config::mmWaveCommSrv")

add_executable(xwr_config_srv
  src/mmWaveCommSrvNode.cpp
)

target_link_libraries(xwr_config_srv
  xwr_config_srv_class
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    xwr_config_srv
    xwr_config_srv_class
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_package()
