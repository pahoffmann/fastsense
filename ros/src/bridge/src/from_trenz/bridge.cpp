/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <signal.h>
#include <chrono>
#include <ros/ros.h>
#include <bridge/from_trenz/tsdf_bridge.h>
#include <bridge/from_trenz/imu_bridge.h>
#include <bridge/from_trenz/velodyne_bridge.h>
#include <bridge/from_trenz/transform_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from_trenz_bridge");
    ros::NodeHandle n("~");

    std::string board_addr;
    if(!ros::param::get("~board_addr", board_addr))
    {
        board_addr = "192.168.1.123"; // default hardware board
    }
    ROS_INFO_STREAM("Board address is " << board_addr);

    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr};
    fs::bridge::ImuBridge imu_bridge{n, board_addr};
    fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr};
    fs::bridge::TransformBridge transform_bridge{n, board_addr};
    
    tsdf_bridge.start();
    imu_bridge.start();
    velodyne_bridge.start();
    transform_bridge.start();
    
    ROS_INFO_STREAM("from_trenz bridge started");

    ros::Rate rate(1.0);
    while(ros::ok())
    {
        rate.sleep();
    }

    tsdf_bridge.stop();
    imu_bridge.stop();
    velodyne_bridge.stop();
    transform_bridge.stop();

    ROS_INFO_STREAM("from_trenz bridge stopped");

    return 0;
}
