cmake_minimum_required(VERSION 3.0.2)
project(bridge)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
)

find_package(ZeroMQ 4.2.5 REQUIRED)
find_package(cppzmq 4.2.3 REQUIRED)

catkin_package()

set(FASTSENSE_DIR ../../../../../../fastsense)

include_directories(
  ${FASTSENSE_DIR}/src
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tsdf_bridge src/tsdf_bridge.cpp)

target_link_libraries(tsdf_bridge
  ${catkin_LIBRARIES}
  libzmq
)
