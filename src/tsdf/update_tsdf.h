#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <map/local_map.h>
#include <util/types.h>

namespace fastsense::tsdf
{

/**
 * @brief Calculate the new TSDF values and fill the 2d grid position array (for every thread one array)
 *
 * @param cloud Points from which the new TSDF value should be determined
 * @param grid_positions Grid positions considered by every thread. THe array is filled by this function
 */
void update_tsdf(const ScanPoints_t& scan_points,
                 const Vector3i& scanner_pos,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight);

} // namespace fastsense::tsdf
