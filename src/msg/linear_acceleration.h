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
    LinearAcceleration()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    LinearAcceleration(float x, float y, float z)
    :   Vector3f{x, y, z}
    {}

    explicit LinearAcceleration(const double* acceleration)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = -acceleration[0] * fastsense::util::params::G;
        (*this)[1] = -acceleration[1] * fastsense::util::params::G;
        (*this)[2] = -acceleration[2] * fastsense::util::params::G;
    }
};

} // namespace fastsense::msg
