#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <map/local_map.h>
#include <hw/buffer/buffer.h>
#include <util/point_hw.h>

namespace fastsense::tsdf
{

/**
 * @brief Generate new TSDF data and weights based on the new point data and update the given map
 *        This allgorithm is equivalent to the normql update_tsdf function.
 *        The difference is, that this function only uses hardware structs (like PointHW)
 * 
 * @param scan_points Points from which the TSDF values should be calculated
 * @param buffer map which should be updated with the new data
 * @param tau Truncation distance for the TSDF values (in map resolution)
 * @param max_weight Maximum weight for every TSDF cell in the map 
 */
void update_tsdf_hw(const fastsense::buffer::InputBuffer<PointHW>& scan_points,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight);

} // namespace fastsense::tsdf
