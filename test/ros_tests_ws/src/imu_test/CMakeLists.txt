cmake_minimum_required(VERSION 3.0.2)
project(imu_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

set(FASTSENSE_DIR ../../../../../fastsense)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FASTSENSE_DIR}/src/driver/imu 
  ${FASTSENSE_DIR}/src
)

add_executable(${PROJECT_NAME}_node
  ${FASTSENSE_DIR}/src/driver/imu/api/phidget.cpp
  ${FASTSENSE_DIR}/src/msg/linear_acceleration.cpp
  ${FASTSENSE_DIR}/src/msg/angular_velocity.cpp
  ${FASTSENSE_DIR}/src/msg/magnetic_field.cpp
  ${FASTSENSE_DIR}/src/msg/imu_msg.cpp
  ${FASTSENSE_DIR}/src/driver/imu/imu.cpp
  ${FASTSENSE_DIR}/src/util/time_stamp.cpp
  ${FASTSENSE_DIR}/src/util/process_thread.cpp
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  phidget21
)

