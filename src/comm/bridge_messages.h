/**
 * @file bridge_messages.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <tsdf/TSDFParameters.h>
#include <vector>

namespace fastsense::comm 
{

struct TSDFBridgeMessage 
{
    float tau_;
    float map_resolution_;
    fastsense::util::LocalMapSize size_;
    fastsense::util::LocalMapPos pos_;
    fastsense::util::LocalMapOffset offset_;
    std::vector<std::pair<float, float>> tsdf_data_;
};

} // namespace fastsense::comm
