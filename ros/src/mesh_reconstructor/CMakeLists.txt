cmake_minimum_required(VERSION 3.0.2)
project(mesh_reconstructor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(LVR2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_filters
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# External interface for HDF5 (libhdf5) HighFive (https://github.com/BlueBrain/HighFive).
set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")
if (NOT TARGET HighFive)
    add_subdirectory(../../../ext/HighFive build)
endif()
include_directories("../../../ext/HighFive/include")
find_package(HDF5 REQUIRED COMPONENTS CXX C HL)
include_directories(${HDF5_INCLUDE_DIRS})


function(MAKE_NODE NAME CPP)
  #add_executable(${NAME} ${CPP} ${SOURCES})
  add_executable(${NAME} ${CPP})

  target_link_libraries(${NAME}
    ${catkin_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    Eigen3::Eigen
    ${LVR2_LIBRARIES}
  )
endfunction()

make_node(mesh_reconstructor src/mesh_reconstructor.cpp)
