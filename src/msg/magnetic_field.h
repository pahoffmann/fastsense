#pragma once

/**
 * @file magnetic_field.h
 * @author Julian Gaal
 */

#include <util/point.h>

namespace fastsense::msg
{

/**
 * @brief Represents magnetic field data from imu
 */
struct MagneticField : public Vector3f
{
    MagneticField() = default;
    explicit MagneticField(const double* magneticField);
};

} // namespace fastsense::msg