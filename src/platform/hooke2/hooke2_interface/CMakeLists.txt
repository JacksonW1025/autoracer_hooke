cmake_minimum_required(VERSION 3.5)
project(hooke2_interface)

find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/common
)

ament_auto_add_library(hooke2_data_converter SHARED
  src/common/throttle_command_100.cpp
  src/common/brake_command_101.cpp
  src/common/steering_command_102.cpp
  src/common/gear_command_103.cpp
  src/common/park_command_104.cpp
  src/common/vehicle_mode_command_105.cpp

  include/common/throttle_command_100.hpp
  include/common/brake_command_101.hpp
  include/common/steering_command_102.hpp
  include/common/gear_command_103.hpp
  include/common/park_command_104.hpp
  include/common/vehicle_mode_command_105.hpp

  include/common/protocol_data.hpp
  include/common/message_manager.hpp
  include/common/param_monitor.hpp
)
target_link_libraries(hooke2_data_converter fmt::fmt)

ament_auto_add_executable(hooke2_interface
  src/hooke2_interface/hooke2_interface.cpp
  src/hooke2_interface/hooke2_interface_node.cpp
  src/hooke2_message_manager/hooke2_message_manager.cpp
  include/common/param_monitor.hpp
)

target_link_libraries(hooke2_interface hooke2_data_converter)

ament_auto_add_executable(hooke2_diag_publisher
  src/hooke2_interface/hooke2_diag_publisher.cpp
  src/hooke2_interface/hooke2_diag_publisher_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
