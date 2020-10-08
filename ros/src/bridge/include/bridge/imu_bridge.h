#pragma once

/**
 * @file imu_bridge.h
 * @author Julian Gaal
 * @date 2020-09-30
 */

#include <array>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "bridge_base.h"
#include <msg/imu_msg.h>
#include <util/process_thread.h>

namespace fastsense::bridge
{

class ImuBridge :   public BridgeBase<msg::ImuMsg, sensor_msgs::Imu, 5555>, 
                    public util::ProcessThread
{
public:
    ImuBridge() = delete;
    ImuBridge(ros::NodeHandle& n, const std::string& board_addr);
    ~ImuBridge() = default;

    void start() override;
    void stop() override;
private:
    void publish() override;
    void convert() override;
    void run() override;

    void initCovariance();

    sensor_msgs::Imu imu_ros_;
    sensor_msgs::MagneticField mag_ros_;
    ros::Publisher mag_pub_;

    /// angular velocity covariance
    std::array<double, 9> angular_velocity_covariance_;

    /// linear acceleration covariance
    std::array<double, 9> linear_acceleration_covariance_;

    /// magnetic field covariance
    std::array<double, 9> magnetic_field_covariance_;
};

} // namespace fastsense::bridge