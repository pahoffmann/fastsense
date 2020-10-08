#pragma once

/**
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/xyz_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents linear acceleration data from imu
 */
struct LinearAcceleration : public util::XYZBuffer<double>
{
    LinearAcceleration() = default;
    explicit LinearAcceleration(const double* acceleration)
    {
        data_[0] = -acceleration[0] * fastsense::util::params::G;
        data_[1] = -acceleration[1] * fastsense::util::params::G;
        data_[2] = -acceleration[2] * fastsense::util::params::G;
    }

};

} // namespace fastsense::msg
