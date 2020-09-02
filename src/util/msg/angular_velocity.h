/**
 * @file angular_velocity.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

#include "../params.h"
#include "xyz.h"

namespace fastsense::util::msg
{

/**
 * @brief Represents angular velocity data from imu
 */
struct AngularVelocity : public XYZ
{
    AngularVelocity() = default;
    AngularVelocity(const double* angular_rate);
};

} // namespace fastsense::util::msg
