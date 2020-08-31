/**
 * @file imu_msg.h
 * @author Julian Gaal
 * @date 2020-08-18
 */

#pragma once

#include "../params.h"
#include "linear_acceleration.h"
#include "angular_velocity.h"
#include "magnetic_field.h"
#include <iostream>
#include <cmath>
#include <memory>

namespace fastsense
{
namespace driver
{
namespace msg
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

} // namespace msg
} // namespace driver
} // namespace fastsense

std::ostream& operator<<(std::ostream& os, const fastsense::driver::msg::ImuMsg& data);
