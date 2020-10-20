/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <chrono>
#include <ros/ros.h>
#include <bridge/tsdf_bridge.h>
#include <bridge/imu_bridge.h>
#include <bridge/velodyne_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

// TODO skalierung
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;

    std::string board_addr;
    if(!ros::param::get("~board_addr", board_addr))
    {
        board_addr = "192.168.1.123"; // default hardware board
    }
    std::cout << "Board address is \"" << board_addr << "\"\n";

    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr};
    //fs::bridge::ImuBridge imu_bridge{n, board_addr};
    //fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr};
    
    tsdf_bridge.start();
    //imu_bridge.start();
    //velodyne_bridge.start();

    while(ros::ok())
    {
        std::this_thread::sleep_for(1s);
    }

    tsdf_bridge.stop();
    //imu_bridge.stop();
    //velodyne_bridge.stop();

    return 0;
}
