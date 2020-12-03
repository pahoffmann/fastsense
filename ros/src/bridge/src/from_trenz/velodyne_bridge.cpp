/**
 * @file velodyne_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <chrono>
#include <iterator>
#include <ros/ros.h>
#include <bridge/from_trenz/velodyne_bridge.h>

using namespace fastsense::bridge;

VelodyneBridge::VelodyneBridge(ros::NodeHandle& n, const std::string& board_addr)
    :   BridgeBase{n, "velodyne_bridge/from_trenz/points", board_addr},
        ProcessThread{},
        points_{}
{
}

void VelodyneBridge::run()
{
    while (running && ros::ok())
    {
        BridgeBase::run();
        ROS_INFO_STREAM("Received " << msg_.data_.points_.size() << " points\n");
    }
}

void VelodyneBridge::convert()
{
    points_.clear();

    const auto& data = msg_.data_;

    auto t_since_epoch = msg_.timestamp_.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(t_since_epoch).count();
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_since_epoch).count() % 1'000'000;
    timestamp_ = ros::Time(sec, nsec);

    std::transform(data.points_.begin(), data.points_.end(), std::back_inserter(points_), [](const ScanPoint& p)
    {
        geometry_msgs::Point32 out;
        out.x = p.x() * 0.001f;
        out.y = p.y() * 0.001f;
        out.z = p.z() * 0.001f;
        return out;
    });

    ROS_INFO_STREAM("Converted points: " << data.points_.size() << "->" << points_.size() << " points\n");
}

void VelodyneBridge::publish()
{
    sensor_msgs::PointCloud pc;
    pc.header.stamp = timestamp_;
    pc.header.frame_id = "base_link";
    pc.points = points_;
    pub().publish(pc);

    ROS_INFO_STREAM("Published points values\n");
}