/**
 * @file bridge_messages.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <util/xyz_buffer.h>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace fastsense::comm 
{

struct TSDFBridgeMessage
{
    float tau_;
    std::array<int, 3> size_;
    std::array<int, 3> pos_;
    std::array<int, 3> offset_;
    std::vector<std::pair<float, float>> tsdf_data_;
};

} // namespace fastsense::comm
