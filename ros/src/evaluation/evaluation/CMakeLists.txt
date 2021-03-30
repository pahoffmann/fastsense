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

add_executable(pose_saver_subscriber src/pose_saver_subscriber.cpp)
target_include_directories(pose_saver_subscriber PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(pose_saver_subscriber PRIVATE ${catkin_LIBRARIES})
add_dependencies(pose_saver_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(loam_to_pose_node src/loam_pcl_to_pose.cpp)
target_include_directories(loam_to_pose_node PRIVATE ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(loam_to_pose_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(loam_to_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})