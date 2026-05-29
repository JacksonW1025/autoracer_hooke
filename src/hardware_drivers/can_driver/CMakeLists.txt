cmake_minimum_required(VERSION 3.15)
project(can_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find_package(autoware_cmake REQUIRED)
# autoware_package()

find_package(ament_cmake_auto  REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

# 因为ESD can与KVASER can的库文件冲突，所以只能二选一，也可以都不编
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

# 是否将库安装置系统中，默认不安装，如果需要安装请使用sudo，如果没有将库拷贝至系统过的'/usr/local/lib'目录，安装会解决执行时找不到库问题
set(INSTALL_CAN_LIB OFF)
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
  # can_client
)

target_link_libraries(can_driver_node
# #   ${Boost_LIBRARIES}
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
  launch)

# set_target_properties(${PROJECT_NAME} PROPERTIES
#   # INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
#   INSTALL_RPATH can_client/third_lib/esd_can/lib/64
#   can_client/third_lib/innodisk/lib/64
# )
