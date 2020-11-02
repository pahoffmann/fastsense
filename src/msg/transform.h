#pragma once

/**
 * @file transform.h
 * @author Marcel Flottmann
 */

#include <util/types.h>

namespace fastsense::msg
{

struct Transform
{
    Transform() = default;
    ~Transform() = default;

    Eigen::Quaternionf rotation;
    Vector3f translation;
};

} // namespace fastsense::msg