cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_pid_controller test/test_pid_controller.py)
  ament_add_pytest_test(test_gpattern_generator test/test_gpattern_generator.py)
  ament_add_pytest_test(test_inverse_kinematics test/test_inverse_kinematics.py)
endif()

ament_package()
