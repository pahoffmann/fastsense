cmake_minimum_required(VERSION 3.0.2)
project(bridge)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
)

find_package(ZeroMQ 4.3.2 REQUIRED)
find_package(cppzmq 4.4.1 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX C HL)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

set(FASTSENSE_DIR ../../../../fastsense)                               
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)
set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")

add_subdirectory(${FASTSENSE_DIR}/ext/HighFive ${CMAKE_CURRENT_BINARY_DIR}/HighFive)

catkin_package()

add_executable(bridge 
  ${FASTSENSE_DIR}/src/msg/imu.cpp
  ${FASTSENSE_DIR}/src/msg/magnetic_field.cpp
  src/imu_bridge.cpp
  src/tsdf_bridge.cpp
  src/velodyne_bridge.cpp
  src/bridge.cpp
  src/transform_bridge.cpp
)

target_compile_options(bridge PRIVATE ${OpenMP_FLAGS})

target_include_directories(bridge PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_DIR}/ext/HighFive/include 
  include
  ${catkin_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(bridge
  ${catkin_LIBRARIES}
  libzmq
  ${OpenMP_LIBS}
)
