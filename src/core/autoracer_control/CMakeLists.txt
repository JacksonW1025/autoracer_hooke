cmake_minimum_required(VERSION 3.14)
project(autoracer_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
