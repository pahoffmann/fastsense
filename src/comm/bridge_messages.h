/**
 * @file bridge_messages.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <tsdf/TSDFParameters.h>
#include <util/xyz_buffer.h>
#include <vector>

namespace fastsense::comm 
{

struct TSDFBridgeMessage 
{
    TSDFBridgeMessage() = default;
    ~TSDFBridgeMessage() = default;
    float tau_;
    float map_resolution_;
    fastsense::util::LocalMapSize size_;
    fastsense::util::LocalMapPos pos_;
    fastsense::util::LocalMapOffset offset_;
    std::vector<std::pair<float, float>> tsdf_data_;
};

} // namespace fastsense::comm
