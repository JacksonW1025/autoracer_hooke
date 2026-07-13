cmake_minimum_required(VERSION 3.15)
project(can_client)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  innodisk
  socket
  third_lib/innodisk_can/include
)

find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  can_client.hpp
  socket/socket_can_client.hpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(COMPILER_INNODISK)
  add_library(innodisk_lib SHARED
    innodisk/innodisk_can_client.hpp
  )
  set_target_properties(innodisk_lib PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(innodisk_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/innodisk_can/lib/64/lib_emuc2_64.so
  )
  target_link_libraries(${PROJECT_NAME}
    innodisk_lib
  )
elseif(COMPILER_ESD)
  add_library(esd_lib SHARED
    esd/esd_can_client.hpp
  )
  set_target_properties(esd_lib PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(esd_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/esd_can/lib/64/libntcan.so.4.2.9
  )
  target_link_libraries(${PROJECT_NAME}
    esd_lib
  )
elseif(COMPILER_KVASER)
  add_library(kvaser_lib SHARED
    kvaser/kvaser_can_client.hpp
  )
  set_target_properties(kvaser_lib PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(kvaser_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/kvaser_can/lib/libcanlib.so
  )
  target_link_libraries(${PROJECT_NAME}
    kvaser_lib
  )
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
  third_lib
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  can_msgs
)
