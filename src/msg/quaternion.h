#pragma once

/**
 * @file quaternion.h
 * @author Julian Gaal
 */

#include <util/xyz_buffer.h>

namespace fastsense::msg
{

struct Quaternion : public util::XYZBuffer<double>
{
    Quaternion() = default;

    Quaternion(double val) : XYZBuffer(val), w(val)
    {
    }

    Quaternion(double x, double y, double z, double w) : XYZBuffer(x, y, z), w(w)
    {
    }

    ~Quaternion() = default;

    double w;
};

} // namespace fastsense::msg