/**
 * @file tsdf_bridge.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "bridge_base.h"
#include <msg/tsdf_bridge_msg.h>

namespace fastsense::bridge
{

/**
 * @brief TSDFBridge converts TSDF data of type msg::TSDFBridgeMessage into ROS Markers
 */
class TSDFBridge :  public BridgeBase<msg::TSDFBridgeMessage, visualization_msgs::Marker, 6666>, 
                    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new TSDFBridge object
     * 
     * @param n nodehandle
     * @param board_addr ip addr of Trenz board
     */
    TSDFBridge(ros::NodeHandle& n, const std::string& board_addr);

    /**
     * @brief Destroy the TSDFBridge object
     */
    ~TSDFBridge() override = default;
private:

    /**
     * @brief Publishes a visualization_msgs::Marker with TSDF values (convert() FIRST for newest data)
     */
    void publish() override;

    /**
     * @brief Converts msg::TSDFBridgeMessage to visualization_msgs::Marker
     */
    void convert() override;

    /**
     * @brief Run listens for TSDFBridgeMessages, converts it to ROS Marker
     * and publishes in an endless loop (running in its own thread)
     */
    void run() override;

    void thread_run() override
    {
        run();
    }

    bool in_bounds(int x, int y, int z);
    std::pair<int, int> get_tsdf_value(int x, int y, int z);

    std::vector<geometry_msgs::Point> points_;
    std::vector<std_msgs::ColorRGBA> colors_;
};

} // namespace fastsense::bridge