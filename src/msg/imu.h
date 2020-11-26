#pragma once

/**
 * @file imu.h
 * @author Julian Gaal
 */

#include <iostream>
#include <memory>

#include <util/time.h>
#include <msg/linear_acceleration.h>
#include <msg/angular_velocity.h>
#include <msg/magnetic_field.h>

#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents data from imu, similar to ROS sensor_msgs/Imu
 */
struct Imu
{
    Imu()
    : acc{}
    , ang{}
    , mag{}
    {}

    Imu(const double* acceleration, const double* angular_rate, const double* magField)
    : acc{acceleration}
    , ang{angular_rate}
    , mag{magField}
    {}

    Imu(LinearAcceleration acc, AngularVelocity ang, MagneticField mag)
    : acc{std::move(acc)}
    , ang{std::move(ang)}
    , mag{std::move(mag)}
    {}

    LinearAcceleration acc;
    AngularVelocity ang;
    MagneticField mag;

    using ptr = std::shared_ptr<Imu>;
};

using ImuStamped = std::pair<Imu, util::HighResTimePoint>;
using ImuStampedBuffer = util::ConcurrentRingBuffer<ImuStamped>;
using ImuStampedBufferPtr = std::shared_ptr<ImuStampedBuffer>;

} // namespace fastsense::msg

//std::ostream& operator<<(std::ostream& os, const fastsense::msg::Imu& data)
//{
//    os << "-- acc --\n";
//    os << data.acc.x() << "\n";
//    os << data.acc.y() << "\n";
//    os << data.acc.z() << "\n";
//
//    os << "-- ang --\n";
//    os << data.ang.x() << "\n";
//    os << data.ang.y() << "\n";
//    os << data.ang.z() << "\n";
//
//    os << "-- mag --\n";
//    os << data.mag.x() << "\n";
//    os << data.mag.y() << "\n";
//    os << data.mag.z() << "\n";
//
//    return os;
//}