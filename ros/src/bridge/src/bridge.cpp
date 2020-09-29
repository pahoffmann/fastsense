/**
 * @file sender.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <ros/ros.h>
#include <bridge/tsdf_bridge.h>

namespace fs = fastsense;

// TODO skalierung
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;

    fs::bridge::TSDFBridge bridge{n};
    bridge.start();

    return 0;
}
