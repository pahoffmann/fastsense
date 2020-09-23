#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <types.h>
#include <ring_buffer/ring_buffer.h>

namespace fastsense::tsdf
{

enum TsdfCalculation
{
    PROJECTION = 0,
    AVERAGE = 1,
    RANSAC = 2,
    ITERATIVE = 3,
    PROJECTION_INTER = 10,
    AVERAGE_INTER = 11,
    RANSAC_INTER = 12,
    ITERATIVE_INTER = 13,
};

/**
 * @brief Calculate the new TSDF values and fill the 2d grid position array (for every thread one array)
 *
 * @param cloud Points from which the new TSDF value should be determined
 * @param grid_positions Grid positions considered by every thread. THe array is filled by this function
 */
void update_tsdf(const ScanPoints_t<Vector3>& scan_points,
                 const Vector3& scanner_pos,
                 RingBuffer<std::pair<float, float>>& buffer,
                 TsdfCalculation tsdf_method,
                 float tau,
                 float max_weight);

} // namespace fastsense::tsdf