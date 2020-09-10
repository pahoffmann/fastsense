/**
 * @file imu_msg.h
 * @author Julian Gaal
 * @date 2020-08-18
 */

#pragma once

#include <util/params.h>
#include <msg/linear_acceleration.h>
#include <msg/angular_velocity.h>
#include <msg/magnetic_field.h>
#include <iostream>
#include <cmath>
#include <memory>

namespace fastsense::msg
{

/**
 * @brief Represents data from imu, similar to ROS sensor_msgs/Imu
 */
struct ImuMsg
{
    ImuMsg();
    ImuMsg(const double* acc, const double* ang, const double* magField);
    LinearAcceleration acc;
    AngularVelocity ang;
    MagneticField mag;

    using ptr = std::shared_ptr<ImuMsg>;
};

} // namespace fastsense::msg

std::ostream& operator<<(std::ostream& os, const fastsense::msg::ImuMsg& data);