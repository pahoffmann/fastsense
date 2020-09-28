/**
 * @file bridge_messages.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <tsdf/TSDFParameters.h>

namespace fastsense::comm 
{

struct TSDFBridgeMessage 
{
    float tau_;
    float map_resolution_;
    fastsense::map::LocalMapSize size_;
    fastsense::map::LocalMapPos pos_;
    fastsense::map::LocalMapOffset offset_;
    std::array<std::pair<float, float>> tsdf_data_;
};

} // namespace fastsense::comm
