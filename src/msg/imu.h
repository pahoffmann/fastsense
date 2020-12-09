#pragma once

/**
 * @file imu.h
 * @author Julian Gaal
 */

#include <msg/stamped.h>
#include <msg/linear_acceleration.h>
#include <msg/angular_velocity.h>
#include <msg/magnetic_field.h>
#include <msg/stamped.h>

#include <iostream>
#include <memory>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents data from imu, similar to ROS sensor_msgs/Imu
 */
struct Imu
{
    /**
     * @brief Construct a new Imu object (default)
     */
    Imu()
    : acc{}
    , ang{}
    , mag{}
    {}

    /**
     * @brief Construct a new Imu object
     * 
     * @param acceleration linear acceleration, c style array for compatibility with phidget driver
     * @param angular_rate rotational change rate, c style array for compatibility with phidget driver
     * @param magField magnetic field, c style array for compatibility with phidget driver
     */
    Imu(const double* acceleration, const double* angular_rate, const double* magField)
    : acc{acceleration}
    , ang{angular_rate}
    , mag{magField}
    {}

    /**
     * @brief Construct a new Imu object
     * 
     * @param acc linear acceleration
     * @param ang rotational change rate
     * @param mag magnetic field
     */
    Imu(LinearAcceleration acc, AngularVelocity ang, MagneticField mag)
    : acc{std::move(acc)}
    , ang{std::move(ang)}
    , mag{std::move(mag)}
    {}

    /// linear acceleration
    LinearAcceleration acc;

    /// angular velocity
    AngularVelocity ang;

    /// magnetic field
    MagneticField mag;

    using Ptr = std::shared_ptr<Imu>;
};

using ImuStamped = msg::Stamped<Imu>;
using ImuStampedBuffer = util::ConcurrentRingBuffer<ImuStamped>;

} // namespace fastsense::msg

/**
 * @brief pretty print imu data
 * 
 * @param os ostream to print into 
 * @param data imu data
 * @return std::ostream& ostream to return 
 */
inline std::ostream& operator<<(std::ostream& os, const fastsense::msg::Imu& data)
{
   os << "-- acc --\n";
   os << data.acc.x() << "\n";
   os << data.acc.y() << "\n";
   os << data.acc.z() << "\n";

   os << "-- ang --\n";
   os << data.ang.x() << "\n";
   os << data.ang.y() << "\n";
   os << data.ang.z() << "\n";

   os << "-- mag --\n";
   os << data.mag.x() << "\n";
   os << data.mag.y() << "\n";
   os << data.mag.z() << "\n";

   return os;
}