/**
 * @file linear_acceleration.h
 * @author Julian Gaal
 * @date 2020-08-18
 */

#pragma once

#include "xyz.h"

namespace fastsense
{
namespace driver
{
namespace msg
{

/**
 * @brief Represents linear acceleration data from imu
 */
struct LinearAcceleration : public XYZ
{
    LinearAcceleration() = default;
    explicit LinearAcceleration(const double* acceleration);
};

} // namespace fastsense
} // namespace driver
} // namespace msg
