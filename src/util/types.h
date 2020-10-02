#pragma once

/**
 * @author Malte Hillmann
 */

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include <msg/point.h>

using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix4i;
using Eigen::Matrix4f;
using ScanPoints_t = std::vector<Vector3i>;

constexpr int MAP_SHIFT = 6; 							// bitshift for a faster way to apply MAP_RESOLUTION
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT; 			// Resolution of the Map in Millimeter per Cell

constexpr int WEIGHT_SHIFT = 6; 						// bitshift for a faster way to apply WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 1 << WEIGHT_SHIFT; 	// Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION

constexpr int MATRIX_SHIFT = 10; 						// bitshift for a faster way to apply MATRIX_RESOLUTION
constexpr int MATRIX_RESOLUTION = 1 << MATRIX_SHIFT; 	// Resolutions of calculations with Matrixes

static const Vector3i INVALID_POINT(0, 0, 0);

/**
 * @brief calculates floor(a / b), except that a is a Vector of integers and b is a power of 2
 * 
 * @param a the numerator
 * @param shift the power of the denominator b, such that b = 2^shift
 * @return floor((float)a / pow(2, shift))
 */
static inline Vector3i floor_shift(const Vector3i& a, int shift)
{
    return Vector3i(
        a[0] >> shift,
        a[1] >> shift,
        a[2] >> shift
    );
}
