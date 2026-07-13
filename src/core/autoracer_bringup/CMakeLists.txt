cmake_minimum_required(VERSION 3.8)
project(autoracer_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(autoracer_platform_boundary test/test_platform_boundary.py)
endif()

ament_package()
