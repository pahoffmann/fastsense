/**
 * @file angular_velocity.h
 * @author Julian Gaal
 * @date 2020-08-17
 */

#pragma once

#include <util/params.h>
#include <msg/xyz_buffer.h>

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
};

} // namespace fastsense::msg
