/**
 * @file sender.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <chrono>
#include <ros/ros.h>
#include <bridge/tsdf_bridge.h>
#include <bridge/imu_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

// TODO skalierung
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;

    fs::bridge::TSDFBridge tsdf_bridge{n};
    fs::bridge::ImuBridge imu_bridge{n};
    
    tsdf_bridge.start();
    imu_bridge.start();

    while(ros::ok())
    {
        std::this_thread::sleep_for(1s);
    }

    tsdf_bridge.stop();
    imu_bridge.stop();

    return 0;
}
