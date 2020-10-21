#pragma once

/**
 * @file update_tsdf_hw.h
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <map/local_map.h>
#include <hw/buffer/buffer.h>
#include <util/point_hw.h>

namespace fastsense::tsdf
{

/**
 * @brief Calculate the new TSDF values and fill the 2d grid position array (for every thread one array)
 *
 * @param cloud Points from which the new TSDF value should be determined
 * @param grid_positions Grid positions considered by every thread. THe array is filled by this function
 */
void update_tsdf_hw(const fastsense::buffer::InputBuffer<PointHW>& scan_points,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight);

} // namespace fastsense::tsdf
