/**
 * @file velodyne_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <chrono>
#include <iterator>
#include <ros/ros.h>
#include <bridge/util.h>
#include <bridge/from_trenz/lidar_bridge.h>

using namespace fastsense::bridge;

LidarBridge::LidarBridge(ros::NodeHandle& n, const std::string& board_addr, std::chrono::milliseconds timeout, bool discard_timestamp)
    :   BridgeBase{n, "velodyne/points", board_addr, 1000, timeout},
        ProcessThread{},
        cloud_{},
        discard_timestamp_{discard_timestamp}
{
}

void LidarBridge::run()
{
    while (running && ros::ok())
    {
        try
        {
            if (receive())
            {
                ROS_DEBUG_STREAM("Received " << msg_.data_.points_.size() << " points");
                convert();
                publish();
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("velo bridge error: " << e.what());
        }
    }
}

void LidarBridge::convert()
{
    timestamp_ = timestamp_to_rostime(msg_.timestamp_, discard_timestamp_);
    const auto& msg_points = msg_.data_.points_;
    const auto& scaling = msg_.data_.scaling_;
    const auto& h = msg_.data_.height_;
    const auto& w = msg_.data_.width_;

    cloud_.clear();
    cloud_.resize(h * w);

    for (int u = 0; u < h; u++) 
    {
        for (int v = 0; v < w; v++) 
        {
            const auto& p = msg_points[u * w + v];
            cloud_[u * w + v] = PointOuster{p.x() * 0.001f / scaling, p.y() * 0.001f / scaling, p.z() * 0.001f / scaling, 1.0f};
        }
    }
}

void LidarBridge::publish()
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud_, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = timestamp_;

    pub().publish(msg);

    ROS_DEBUG_STREAM("Published points values\n");
}