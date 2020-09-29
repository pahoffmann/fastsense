#pragma once

/**
 * @author Julian Gaal
 */

#include <msg/xyz_buffer.h>

namespace fastsense::msg
{

struct Quaternion : public XYZBuffer<double>
{
    Quaternion() = delete;

    Quaternion(double x, double y, double z, double w);

    ~Quaternion() = default;

    double w;
};

} // namespace fastsense::msg