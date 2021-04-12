cmake_minimum_required(VERSION 3.0.2)
project(evaluation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(PCL REQUIRED)

add_message_files(
    FILES
    SavePoseStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES evaluation
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

add_executable(save_poses_node src/save_poses_node.cpp)
target_include_directories(save_poses_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(save_poses_node PRIVATE ${catkin_LIBRARIES})
add_dependencies(save_poses_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(loam_to_pose_node src/loam_to_pose_node.cpp)
target_include_directories(loam_to_pose_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(loam_to_pose_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(loam_to_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(imu_interpolation_node src/interpolate_imu.cpp)
target_include_directories(imu_interpolation_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(imu_interpolation_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(imu_interpolation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(live_imu_interpolation_node src/live_interpolate_imu.cpp)
target_include_directories(live_imu_interpolation_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(live_imu_interpolation_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(live_imu_interpolation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(reduce_pcl_node src/reduce_pcl.cpp)
target_include_directories(reduce_pcl_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(reduce_pcl_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(reduce_pcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})