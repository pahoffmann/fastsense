#pragma once

/**
 * @file constants.h
 * @author Malte Hillmann
 */

constexpr int MAP_SHIFT = 6;                            // bitshift for a faster way to apply MAP_RESOLUTION
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT;          // Resolution of the Map in Millimeter per Cell

constexpr int WEIGHT_SHIFT = 5;                         // bitshift for a faster way to apply WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 1 << WEIGHT_SHIFT;    // Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION

constexpr int MATRIX_SHIFT = 15;                        // bitshift for a faster way to apply MATRIX_RESOLUTION
constexpr int MATRIX_RESOLUTION = 1 << MATRIX_SHIFT;    // Resolution of calculations (Matrices, division, ...)

constexpr int RINGS = 16; // TODO: take from Scanner
//#include <math.h>
//const int dz_per_distance = std::tan(30.0 / ((double)RINGS - 1.0) / 180.0 * M_PI) / 2.0 * MATRIX_RESOLUTION;
constexpr int dz_per_distance = 572;
