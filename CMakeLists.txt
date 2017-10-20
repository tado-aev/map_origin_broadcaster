cmake_minimum_required(VERSION 2.8.3)
project(map_origin_broadcaster)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  gnss
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_origin_broadcaster
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(map_origin_broadcaster
  src/map_origin_broadcaster.cpp
  include/map_origin_broadcaster/map_origin_broadcaster.h
)

add_executable(map_origin_broadcaster_node src/map_origin_broadcaster_node.cpp)

target_link_libraries(map_origin_broadcaster_node
  map_origin_broadcaster
  ${catkin_LIBRARIES}
)

install(TARGETS map_origin_broadcaster map_origin_broadcaster_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
