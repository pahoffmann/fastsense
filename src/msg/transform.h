#pragma once

/**
 * @file transform.h
 * @author Marcel Flottmann
 */

#include <msg/quaternion.h>
#include <util/xyz_buffer.h>

namespace fastsense::msg
{

struct Transform
{
    Transform() = default;
    ~Transform() = default;

    Quaternion rotation;
    util::XYZBuffer<double> translation;
};

} // namespace fastsense::msg