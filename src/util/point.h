#pragma once

/**
 * @file point.h
 * @author Malte Hillmann
 */

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

namespace fastsense
{

using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix4i;
using Eigen::Matrix4f;
using Eigen::Quaternionf;
using ScanPoints_t = std::vector<Vector3i>;
using ScanPointType = int16_t;
using ScanPoint = Eigen::Matrix<ScanPointType, 3, 1>;

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

} // namespace fastsense
