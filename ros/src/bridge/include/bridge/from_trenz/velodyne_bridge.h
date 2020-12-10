/**
 * @file velodyne_bridge.h
 * @author Marcel Flottmann
 * @date 2020-09-29
 */

#pragma once

#include <ros/time.h>
#include <sensor_msgs/PointCloud.h>
#include <msg/point_cloud.h>
#include "bridge_base.h"

namespace fastsense::bridge
{

/**
 * @brief VelodyneBridge converts Velodyne data from custom driver running 
 * on Trenz board and publishes a sensor_msgs::PointCloud
 * 
 */
class VelodyneBridge :  public BridgeBase<msg::PointCloudStamped, sensor_msgs::PointCloud, 7777>,
    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new Velodyne Bridge object
     * 
     * @param n nodehandle
     * @param board_addr ip addr of Trenz board
     */
    VelodyneBridge(ros::NodeHandle& n, const std::string& board_addr);

    /**
     * @brief Destroy the Velodyne Bridge object
     */
    ~VelodyneBridge() override = default;
private:

    /**
     * @brief Publishes a sensor_msgs::PointCloud (convert() FIRST for newest data)
     */
    void publish() override;

    /**
     * @brief Converts msg::PointCloud into std::vector<geometry_msgs::Point32>>
     */
    void convert() override;
   
    /**
     * @brief Run listens for lidar data, converts it to ROS PointCloud message
     * and publishes in an endless loop (running in its own thread)
     */
    void run() override;

    void thread_run() override
    {
        run();
    }

    /// Local vector of lidar points that are published
    std::vector<geometry_msgs::Point32> points_;
    ros::Time timestamp_;
};

} // namespace fastsense::bridge