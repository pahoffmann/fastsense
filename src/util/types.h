#pragma once

/**
 * @author Malte Hillmann
 */

#include <eigen3/Eigen/Dense>
#include <vector>

#include <map/local_map.h>
#include <msg/point.h>

using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix4i;
using Eigen::Matrix4f;
using ScanPoints_t = std::vector<Vector3i>;
using LocalMap_t = fastsense::map::LocalMap<std::pair<int, int>>;

constexpr int MAP_SHIFT = 6; 							// bitshift for a faster way to apply MAP_RESOLUTION
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT; 			// Resolution of the Map in Millimeter per Cell

constexpr int WEIGHT_SHIFT = 6; 						// bitshift for a faster way to apply WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 1 << WEIGHT_SHIFT; 	// Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION

constexpr int MATRIX_SHIFT = 10; 						// bitshift for a faster way to apply MATRIX_RESOLUTION
constexpr int MATRIX_RESOLUTION = 1 << MATRIX_SHIFT; 	// Resolutions of calculations with Matrixes

static const Vector3i INVALID_POINT(0, 0, 0);
