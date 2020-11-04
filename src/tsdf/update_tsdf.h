#pragma once

/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <map/local_map.h>

namespace fastsense::tsdf
{

/**
 * @brief Generate new TSDF data and weights based on the new point data and update the given map
 *
 * @param scan_points Points from which the TSDF values should be calculated
 * @param scanner_pos current position of the laser scanner
 * @param buffer map which should be updated with the new data
 * @param tau Truncation distance for the TSDF values (in map resolution)
 * @param max_weight Maximum weight for every TSDF cell in the map
 */
void update_tsdf(const ScanPoints_t& scan_points,
                 const Vector3i& scanner_pos,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight);

} // namespace fastsense::tsdf
