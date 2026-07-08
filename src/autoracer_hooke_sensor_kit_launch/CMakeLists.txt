cmake_minimum_required(VERSION 3.8)
project(autoracer_hooke_sensor_kit_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
