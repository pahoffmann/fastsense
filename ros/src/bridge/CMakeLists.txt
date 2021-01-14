#if (DEFINED ENV{XILINX_VITIS})
#  message(FATAL_ERROR "Can't compile in bash session in which Vitis Tools have been sourced!\nCompile from a clean bash environment")
#endif()

cmake_minimum_required(VERSION 3.3)

project(bridge CXX)
set(CMAKE_CXX_STANDARD 17)

include(cmake/helpers.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  message_filters
  rostest
)

find_package(ZeroMQ 4.3.2 REQUIRED EXACT)
find_package(cppzmq 4.4.1 REQUIRED EXACT)
find_package(HDF5 1.10.0.1 REQUIRED COMPONENTS CXX C HL)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

set_fastsense_root_dir(FASTSENSE_DIR)
get_absolute(FASTSENSE_ROS_INCLUDE_PATH include)
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)
set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")

catkin_package()

add_subdirectory(${FASTSENSE_DIR}/ext/HighFive ${CMAKE_CURRENT_BINARY_DIR}/HighFive)
add_subdirectory(src/from_trenz)
add_subdirectory(src/to_trenz)

add_rostest_gtest(to_trenz_test test/test.launch test/test.cpp)
target_link_libraries(to_trenz_test ${catkin_LIBRARIES})

install(PROGRAMS scripts/sync_time.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
