/**
 * @file
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <iterator>
#include <ros/ros.h>
#include <bridge/transform_bridge.h>

using namespace fastsense::bridge;

TransformBridge::TransformBridge(ros::NodeHandle& n, const std::string& board_addr)
    :   BridgeBase{n, "transform_bridge/pose", board_addr},
        ProcessThread{},
        transform_data{}
{
}

void TransformBridge::start()
{
    if (running == false)
    {
        running = true;
        worker = std::thread(&TransformBridge::run, this);
    }
}

void TransformBridge::stop()
{
    if (running)
    {
        running = false;
        worker.join();
    }
}

void TransformBridge::run()
{
    while (running && ros::ok())
    {
        BridgeBase::run();
    }
}

void TransformBridge::convert()
{
    transform_data.transform.rotation.x = msg_.rotation.x();
    transform_data.transform.rotation.y = msg_.rotation.y();
    transform_data.transform.rotation.z = msg_.rotation.z();
    transform_data.transform.rotation.w = msg_.rotation.w;
    transform_data.transform.translation.x = msg_.translation.x();
    transform_data.transform.translation.y = msg_.translation.y();
    transform_data.transform.translation.z = msg_.translation.z();
}

void TransformBridge::publish()
{
    transform_data.header.stamp = ros::Time::now();
    transform_data.header.frame_id = "map";
    transform_data.child_frame_id = "base_link";

    pub().publish(transform_data);
    ROS_INFO_STREAM("Pose published\n");
}