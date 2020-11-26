#pragma once

/**
 * @file linear_acceleration.h
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/point.h>

namespace fastsense::msg
{

/**
 * @brief Represents linear acceleration data from imu
 */
struct LinearAcceleration : public Vector3f
{
    LinearAcceleration() = default;

    LinearAcceleration(float x, float y, float z)
    :   Vector3f{}
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
    }

    explicit LinearAcceleration(const double* acceleration)
    :   Vector3f{}
    {
        (*this)[0] = -acceleration[0] * fastsense::util::params::G;
        (*this)[1] = -acceleration[1] * fastsense::util::params::G;
        (*this)[2] = -acceleration[2] * fastsense::util::params::G;
    }

};

} // namespace fastsense::msg
