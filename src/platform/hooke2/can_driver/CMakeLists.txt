cmake_minimum_required(VERSION 3.15)
project(can_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

# The deployed Hooke2 path uses SocketCAN. Proprietary backends remain opt-in.
set(COMPILER_INNODISK OFF)
set(COMPILER_ESD OFF)
set(COMPILER_KVASER OFF)

if(COMPILER_INNODISK)
  add_compile_definitions(INNODISK_CAN)
elseif(COMPILER_ESD)
  add_compile_definitions(ESD_CAN)
elseif(COMPILER_KVASER)
  add_compile_definitions(KVASER_CAN)
endif()

include_directories(
  include
  can_client
)

add_subdirectory(can_client)

ament_auto_add_library(can_driver_node SHARED
  src/can_driver.cpp
)

ament_target_dependencies(can_driver_node
  rclcpp
  rclcpp_components
  can_msgs
)

target_link_libraries(can_driver_node
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  can_client
)

rclcpp_components_register_node(can_driver_node
  PLUGIN "can_driver::CanDriver"
  EXECUTABLE can_driver
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
