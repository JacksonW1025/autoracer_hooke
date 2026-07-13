cmake_minimum_required(VERSION 3.8)
project(autoracer_hooke2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    autoracer_hooke2_sensing_boundary
    test/test_sensing_boundary.py
  )
endif()

ament_package()
