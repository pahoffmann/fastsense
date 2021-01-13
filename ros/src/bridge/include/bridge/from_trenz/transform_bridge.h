/**
 * @file velodyne_bridge.h
 * @author Marcel Flottmann
 * @date 2020-09-29
 */

#pragma once

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <msg/transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include "bridge_base.h"

#include <mutex>

namespace fastsense::bridge
{

/**
 * @brief TransformBridge converts transform data from the application to ROS messages
 * 
 */
class TransformBridge :  public BridgeBase<msg::TransformStamped, nav_msgs::Path, 8888>,
    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new Transform Bridge object
     *
     * @param n ros::NodeHandle
     * @param board_addr ip addr of Trenz board
     */
    TransformBridge(ros::NodeHandle& n, const std::string& board_addr);

    /**
     * @brief Destroy the Transform Bridge object
     */
    ~TransformBridge() override = default;

    /**
     * @brief Starts the Transform Bridge in its own thread
     */
    void start() override;

    /**
     * @brief Stops the Transform Bridge thread
     */
    void stop() override;
private:

    /**
     * @brief Publishes a geometry_msgs::TransformStamped (convert() FIRST for newest data)
     */
    void publish() override;

    /**
     * @brief Converts msg::Transform into geometry_msgs::TransformStamped
     */
    void convert() override;
   
    /**
     * @brief Run listens for transform data, converts it to ROS geometry_msgs::TransformStamped message
     * and publishes in an endless loop (running in its own thread)
     */
    void run() override;

    /// Run one "iteration" of thread
    void thread_run() override
    {
        run();
    }

    /// Broadcast transformation
    void broadcast();

    /// Transform broadcaster
    tf2_ros::TransformBroadcaster broadcaster;
    
    /// thread of broadcaster
    std::thread broadcaster_thread;

    /// TODO mutex why though
    std::mutex mtx;

    /// Local vector of lidar points that are published
    geometry_msgs::TransformStamped transform_data;

    /// first message received: used to detect first timestamp
    bool first_smg;

    /// Path message
    nav_msgs::Path pose_path;

    /// Single PoseStamped to append to path
    geometry_msgs::PoseStamped pose_stamped;
};

} // namespace fastsense::bridge