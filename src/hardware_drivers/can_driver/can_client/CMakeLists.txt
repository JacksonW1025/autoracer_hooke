# FILEPATH: /home/duboping/autoware/autoware/src/universe/external/can_driver/can_client/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)
project(can_client)

set(CMAKE_CXX_STANDARD 17)

# Add include directories
include_directories(
  innodisk
  socket
  third_lib/innodisk_can/include
)

# Find required packages
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  can_client.hpp
  socket/socket_can_client.hpp
)
# Set linker language to C++
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX) 

if(COMPILER_INNODISK)
add_library(innodisk_lib SHARED
  innodisk/innodisk_can_client.hpp
)
# Set linker language to C++
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
      ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/esd_can/lib/64/libntcan.so.4.2.9)
target_link_libraries(${PROJECT_NAME}
  esd_lib
)
elseif(COMPILER_KVASER)
add_library(kvaser_lib SHARED
  kvaser/kvaser_can_client.hpp
)
set_target_properties(kvaser_lib PROPERTIES LINKER_LANGUAGE CXX) 
target_link_libraries(kvaser_lib
      ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/kvaser_can/lib/libcanlib.so)
target_link_libraries(${PROJECT_NAME}
  kvaser_lib
)      
endif()

# Add private include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  third_lib
)


# # Link private libraries
# target_link_libraries(${PROJECT_NAME}
#   ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/esd_can/lib/64/libntcan.so.4.2.9
#   ${CMAKE_CURRENT_SOURCE_DIR}/third_lib/innodisk_can/lib/64/lib_emuc2_64.so
# )

# # Add dependencies
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp 
#   can_msgs
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )


file(GLOB ESD_LIB      "${PROJECT_SOURCE_DIR}/third_lib/esd_can/lib/64/*")
file(GLOB INNODISK_LIB "${PROJECT_SOURCE_DIR}/third_lib/innodisk_can/lib/64/*")
file(GLOB KVASER_LIB   "${PROJECT_SOURCE_DIR}/third_lib/kvaser_can/lib/*")

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp 
  can_msgs
)

if (INSTALL_CAN_LIB)
  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES ${ESD_LIB} ${INNODISK_LIB} DESTINATION lib/)
  # 将需要的库文件拷贝至系统目录"/usr/local/lib"，需要sudo权限
  install(FILES ${ESD_LIB} DESTINATION /usr/local/lib)
  install(FILES ${INNODISK_LIB} DESTINATION /usr/local/lib)
  install(FILES ${KVASER_LIB} DESTINATION /usr/local/lib)
  # 执行ldconfig命令
  install(CODE "execute_process(COMMAND sudo ldconfig)")
endif()

# duboping debug 2024-02-07
install(FILES ${ESD_LIB} DESTINATION lib/${PROJECT_NAME})
install(FILES ${INNODISK_LIB} DESTINATION lib/${PROJECT_NAME})
install(FILES ${KVASER_LIB} DESTINATION lib/${PROJECT_NAME})
