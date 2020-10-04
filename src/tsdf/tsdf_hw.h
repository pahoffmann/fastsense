/**
 * @file tsdf_hw.h
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * @brief 
 * @version 0.1
 * @date 2020-10-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

#include <cmath>

// int norm(const Point& point)
// {
//     return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
// }

namespace fastsense::tsdf
{

int norm(int* point)
{
#pragma HLS inline
    return std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
}

} // namespace fastsense::tsdf