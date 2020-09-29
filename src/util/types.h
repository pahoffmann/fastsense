#pragma once

/**
 * @author Malte Hillmann
 */

#include <eigen3/Eigen/Dense>

#include <vector>
#include <msg/point.h>

using Vector3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix3f;
using Mat4 = Eigen::Matrix4f;

template<typename VEC_T>
using ScanPoints_t = std::vector<std::vector<VEC_T>>;

