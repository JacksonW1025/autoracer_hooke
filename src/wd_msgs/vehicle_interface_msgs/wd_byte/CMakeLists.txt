cmake_minimum_required(VERSION 3.15)
project(wd_byte)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
)

ament_auto_add_library(wd_byte SHARED
  src/wd_byte/byte.cpp
)

ament_auto_package()
