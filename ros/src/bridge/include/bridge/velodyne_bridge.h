/**
 * @file velodyne_bridge.h
 * @author Marcel Flottmann
 * @date 2020-09-29
 */

#pragma once

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>

#include "bridge_base.h"
#include <util/process_thread.h>

namespace fastsense::bridge
{

class VelodyneBridge :  public BridgeBase<msg::PointCloud, sensor_msgs::PointCloud, 7777>,
    public util::ProcessThread
{
public:
    VelodyneBridge() = delete;
    VelodyneBridge(ros::NodeHandle& n, const std::string& board_addr);
    ~VelodyneBridge() = default;

    void start() override;
    void stop() override;
private:
    void publish() override;
    void convert() override;
    void run() override;

    std::vector<geometry_msgs::Point32> points_;
};

} // namespace fastsense::bridge