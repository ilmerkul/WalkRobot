cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(python_cmake_module REQUIRED)
  ament_add_pytest_test(test_pid_controller test/test_pid_controller.py)
  ament_add_pytest_test(test_gpattern_generator test/test_gpattern_generator.py)
  ament_add_pytest_test(test_inverse_kinematics test/test_inverse_kinematics.py)
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
