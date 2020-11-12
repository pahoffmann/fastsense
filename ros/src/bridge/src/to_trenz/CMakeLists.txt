add_executable(to_trenz 
  ${FASTSENSE_DIR}/src/msg/imu.cpp
  ${FASTSENSE_DIR}/src/msg/magnetic_field.cpp
  bridge.cpp
)

target_compile_options(to_trenz PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(to_trenz PRIVATE
  ${FASTSENSE_DIR}/src
  ${FASTSENSE_ROS_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(to_trenz
  ${catkin_LIBRARIES}
  libzmq
)