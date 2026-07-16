cmake_minimum_required(VERSION 3.8)
project(autoracer_rc_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(c32_pointcloud_adapter SHARED
  src/c32_pointcloud_adapter.cpp
)
target_include_directories(c32_pointcloud_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(c32_pointcloud_adapter sensor_msgs)

add_executable(c32_pointcloud_adapter_node
  src/c32_pointcloud_adapter_node.cpp
)
target_link_libraries(c32_pointcloud_adapter_node c32_pointcloud_adapter)
ament_target_dependencies(c32_pointcloud_adapter_node rclcpp sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS c32_pointcloud_adapter c32_pointcloud_adapter_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_c32_pointcloud_adapter
    test/test_c32_pointcloud_adapter.cpp
  )
  target_link_libraries(test_c32_pointcloud_adapter c32_pointcloud_adapter)
  ament_target_dependencies(test_c32_pointcloud_adapter sensor_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)
ament_package()
