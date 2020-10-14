#pragma once

/**
 * @file magnetic_field.h
 * @author Julian Gaal
 */

#include <util/xyz_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents magnetic field data from imu
 */
struct MagneticField : public util::XYZBuffer<double>
{
    MagneticField() = default;
    explicit MagneticField(const double* magneticField);
};

} // namespace fastsense::msg