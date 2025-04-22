cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(python_cmake_module REQUIRED)
  ament_add_pytest_test(test_pid_controller test/test_pid_controller.py)
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()