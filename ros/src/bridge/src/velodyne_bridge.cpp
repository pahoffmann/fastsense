/**
 * @file
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <iostream>
#include <iterator>
#include <ros/ros.h>
#include <bridge/velodyne_bridge.h>

using namespace fastsense::bridge;

VelodyneBridge::VelodyneBridge(ros::NodeHandle& n, const std::string& board_addr)
    :   BridgeBase{n, "/velodyne_points", board_addr},
        ProcessThread{},
        points_{}
{
}

void VelodyneBridge::start()
{
    if (running == false)
    {
        running = true;
        worker = std::thread(&VelodyneBridge::run, this);
    }
}

void VelodyneBridge::stop()
{
    if (running)
    {
        running = false;
        worker.join();
    }
}

void VelodyneBridge::run()
{
    while (running && ros::ok())
    {
        BridgeBase::run();
        std::cout << "Received " << msg_.points_.size() << " points\n";
    }
}

void VelodyneBridge::convert()
{
    points_.clear();
    std::transform(msg_.points_.begin(), msg_.points_.end(), std::back_inserter(points_), [](fastsense::msg::Point p)
    {
        geometry_msgs::Point32 out;
        out.x = p.x * 0.001f;
        out.y = p.y * 0.001f;
        out.z = p.z * 0.001f;
        return out;
    });
    std::cout << "Converted points: " << msg_.points_.size() << "->" << points_.size() << " points\n";
}

void VelodyneBridge::publish()
{
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "base_link";
    pc.points = points_;
    pub().publish(pc);

    std::cout << "Published points values\n";
}