/**
 * @file tsdf_bridge.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "bridge_base.h"
#include <util/process_thread.h>
#include <comm/bridge_messages.h>

namespace fastsense::bridge
{

class TSDFBridge :  public BridgeBase<comm::TSDFBridgeMessage, visualization_msgs::Marker, 1111>, 
                    public util::ProcessThread
{
public:
    TSDFBridge() = delete;
    TSDFBridge(ros::NodeHandle& n);
    ~TSDFBridge() = default;

    void start() override;
    void stop() override;
private:
    void publish() override;
    void convert() override;
    void run() override;
    bool inBounds(int x, int y, int z);
    std::pair<float, float> get_tsdf_value(int x, int y, int z);

    std::vector<geometry_msgs::Point> points_;
    std::vector<std_msgs::ColorRGBA> colors_;
};

} // namespace fastsense::bridge