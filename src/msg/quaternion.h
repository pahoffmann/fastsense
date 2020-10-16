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
    Quaternion() = delete;

    Quaternion(double x, double y, double z, double w);

    ~Quaternion() = default;

    double w;
};

} // namespace fastsense::msg