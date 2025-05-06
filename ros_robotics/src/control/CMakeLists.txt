cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(utils)
ament_python_install_package(observation_prepare)

ament_export_dependencies(ament_python)
ament_package()
