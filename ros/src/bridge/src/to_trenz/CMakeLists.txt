add_executable(to_trenz bridge.cpp ${FASTSENSE_DIR}/src/msg/imu.cpp)

target_include_directories(to_trenz PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FASTSENSE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(to_trenz
  ${catkin_LIBRARIES}
  libzmq
)