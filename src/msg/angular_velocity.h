#pragma once

/**
 * @file angular_velocity.h
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/point.h>

constexpr double DEEGREES_TO_RADIANS = M_PI / 180.0;

namespace fastsense::msg
{

/**
 * @brief Represents angular velocity data from imu
 */
struct AngularVelocity : public Vector3f
{
    AngularVelocity()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    AngularVelocity(float x, float y, float z)
    :   Vector3f{x, y, z}
    {}

    AngularVelocity(const double* angular_rate)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = angular_rate[0] * DEEGREES_TO_RADIANS;
        (*this)[1] = angular_rate[1] * DEEGREES_TO_RADIANS;
        (*this)[2] = angular_rate[2] * DEEGREES_TO_RADIANS;
    }
};

} // namespace fastsense::msg
