add_executable(receiver_test
        ${FASTSENSE_DIR}/ros/src/bridge/src/from_trenz/imu_bridge.cpp
        ${FASTSENSE_DIR}/ros/src/bridge/src/from_trenz/transform_bridge.cpp
        ${FASTSENSE_DIR}/ros/src/bridge/src/from_trenz/tsdf_bridge.cpp
        ${FASTSENSE_DIR}/ros/src/bridge/src/from_trenz/velodyne_bridge.cpp
        receiver.cpp
)

target_compile_options(receiver_test PRIVATE ${OpenMP_FLAGS} -Wall -Wextra -Wpedantic)

target_include_directories(receiver_test PRIVATE
        ${FASTSENSE_DIR}/src
        ${FASTSENSE_ROS_INCLUDE_PATH}
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(receiver_test
        ${catkin_LIBRARIES}
        libzmq
        ${OpenMP_LIBS}
)

add_executable(sender_test sender.cpp)

target_include_directories(sender_test PRIVATE ${FASTSENSE_DIR}/src)

target_link_libraries(
        sender_test PRIVATE
        libzmq
)