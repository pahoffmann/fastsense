#pragma once

/**
 * @author Julian Gaal
 */

#include "xyz_buffer.h"

namespace fastsense::msg
{

/**
 * @brief Represents magnetic field data from imu
 */
struct MagneticField : public XYZBuffer<double>
{
    MagneticField() = default;
    explicit MagneticField(const double* magneticField);
};

} // namespace fastsense::msg