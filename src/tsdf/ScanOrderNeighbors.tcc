#pragma once

#include <algorithm>

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

namespace fastsense::tsdf
{

template<typename POINT_T, typename SET_T>
std::vector<POINT_T> ScanOrderNeighbors<POINT_T, SET_T>::operator()(const unsigned short query_ring, const size_t query_index, const SET_T& points) const
{
    std::vector<POINT_T> neighbors;

    int max_ring = points.size() - 1;
    size_t width = points[0].size();
    size_t first_ring = query_ring > neighbor_rings_ ? query_ring - neighbor_rings_ : 0;
    size_t last_ring = std::min(query_ring + neighbor_rings_, max_ring);
    for (int dr = first_ring; dr <= max_ring; dr++)
    {
        for (int di = -neighbor_range_; di <= neighbor_range_; di++)
        {
            const POINT_T& p = points[dr][(query_index + width + di) % width];

            if (!std::isnan(p.x()))
            {
                // TODO: Check Distance
                neighbors.push_back(p);
            }
        }
    }

    return neighbors;
}

} // namespace fastsense::tsdf
