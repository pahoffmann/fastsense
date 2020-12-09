#pragma once

/**
 * @file imu.h
 * @author Julian Gaal
 */

#include <iostream>
#include <memory>

#include <msg/linear_acceleration.h>
#include <msg/angular_velocity.h>
#include <msg/magnetic_field.h>

#include <util/time_stamp.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents data from imu, similar to ROS sensor_msgs/Imu
 */
struct Imu
{
    Imu();
    Imu(const double* acc, const double* ang, const double* magField);
    LinearAcceleration acc;
    AngularVelocity ang;
    MagneticField mag;

    using ptr = std::shared_ptr<Imu>;
};

using ImuStamped = std::pair<Imu, util::TimeStamp>;
using ImuStampedBuffer = util::ConcurrentRingBuffer<ImuStamped>;
using ImuStampedBufferPtr = std::shared_ptr<ImuStampedBuffer>;

} // namespace fastsense::msg

std::ostream& operator<<(std::ostream& os, const fastsense::msg::Imu& data);