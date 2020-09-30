/**
 * @file sender.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <ros/ros.h>
#include <bridge/tsdf_bridge.h>
#include <bridge/imu_bridge.h>

namespace fs = fastsense;

// TODO skalierung
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;

    fs::bridge::TSDFBridge tsdf_bridge{n};
    fs::bridge::ImuBridge imu_bridge{n};
    
    tsdf_bridge.start();
    // imu_bridge.start();

    return 0;
}
