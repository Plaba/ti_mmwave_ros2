cmake_minimum_required(VERSION 3.8)
project(ti_mmwave)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
