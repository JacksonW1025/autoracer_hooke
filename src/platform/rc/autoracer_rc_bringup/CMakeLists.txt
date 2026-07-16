cmake_minimum_required(VERSION 3.8)
project(autoracer_rc_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    autoracer_rc_sensing_boundary
    test/test_sensing_boundary.py
  )
endif()

ament_package()
