#pragma once

/**
 * @file point.h
 * @author Julian Gaal
 */

#include <cstdint>

namespace fastsense::msg
{

/**
 * @brief Represents a point with 16-bit X/Y/Z components
 *
 */
struct Point
{
    /// X component
    int16_t x;

    /// Y component
    int16_t y;

    /// Z component
    int16_t z;

    bool operator==(const Point& p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }
};

} // namespace fastsense::msg
