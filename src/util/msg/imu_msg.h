/**
 * @file imu_msg.h
 * @author Julian Gaal
 * @date 2020-08-18
 */

#pragma once

#include <util/params.h>
#include <util/msg/linear_acceleration.h>
#include <util/msg/angular_velocity.h>
#include <util/msg/magnetic_field.h>
#include <iostream>
#include <cmath>
#include <memory>

namespace fastsense::util::msg
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
};

} // namespace fastsense::util::msg

std::ostream& operator<<(std::ostream& os, const fastsense::util::msg::ImuMsg& data);