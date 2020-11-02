#pragma once

/**
 * @file linear_acceleration.h
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/types.h>

namespace fastsense::msg
{

/**
 * @brief Represents linear acceleration data from imu
 */
struct LinearAcceleration : public Vector3f
{
    LinearAcceleration() = default;
    explicit LinearAcceleration(const double* acceleration)
    {
        (*this)[0] = -acceleration[0] * fastsense::util::params::G;
        (*this)[1] = -acceleration[1] * fastsense::util::params::G;
        (*this)[2] = -acceleration[2] * fastsense::util::params::G;
    }

};

} // namespace fastsense::msg
