/**
 * @file angular_velocity.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

#include <util/params.h>
#include <util/xyz_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents angular velocity data from imu
 */
struct AngularVelocity : public util::XYZBuffer<double>
{
    AngularVelocity() = default;
    AngularVelocity(const double* angular_rate);
};

} // namespace fastsense::msg
