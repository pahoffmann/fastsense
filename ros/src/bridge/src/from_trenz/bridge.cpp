/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */

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

    std::chrono::milliseconds timeout;
    int timeout_ros;
    if(!ros::param::get("~timeout", timeout_ros))
    {
        timeout = std::chrono::milliseconds(static_cast<size_t>(timeout_ros)); // default timeout for receiver
    }

    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr, timeout};
    fs::bridge::ImuBridge imu_bridge{n, board_addr, timeout};
    fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr, timeout};
    fs::bridge::TransformBridge transform_bridge{n, board_addr, timeout};
    
    tsdf_bridge.start();
    imu_bridge.start();
    velodyne_bridge.start();
    transform_bridge.start();
    
    ROS_INFO_STREAM("from_trenz bridge started");

    ros::Rate rate(0.1);
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
