/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <memory>
#include <iostream>
// #include <unordered_map>
#include <utility>

#include "update_tsdf.h"
#include "ScanOrderNeighbors.h"
#include "PriorityNeighbors.h"
#include "AverageNormal.h"
#include "ProjectionNormal.h"
#include "RANSACNormal.h"
#include "IterativeNormal.h"
#include "weighting.h"

// namespace std
// {
// template<> struct hash<Vector3i>
// {
//     std::size_t operator()(Vector3i const& p) const noexcept
//     {
//         long long v = ((long long)p.x() << 32) ^ ((long long)p.y() << 16) ^ (long long)p.z();
//         return std::hash<long long>()(v);
//     }
// };
// }

namespace fastsense::tsdf
{

void update_tsdf(const ScanPoints_t& scan_points,
                 const Vector3i& scanner_pos,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight)
{
    constexpr int RINGS = 16; // TODO: take from Scanner
    int dz_per_distance = std::tan(30.0 / ((double)RINGS - 1.0) / 180.0 * M_PI) / 2.0 * MATRIX_RESOLUTION;

    int weight_epsilon = tau / 10;

    // std::unordered_map<Vector3i, std::pair<int, int>> values;

    for (size_t i = 0; i < scan_points.size(); i++)
    {
        const Vector3i& point = scan_points[i];

        Vector3i direction_vector = point - scanner_pos;
        int distance = direction_vector.norm();

        Vector3i prev(0, 0, 0);
        for (int len = MAP_RESOLUTION; len <= distance + tau; len += MAP_RESOLUTION / 2)
        {
            Vector3i proj = scanner_pos + direction_vector * len / distance;
            Vector3i index = floor_shift(proj, MAP_SHIFT);
            
            if (index.x() == prev.x() && index.y() == prev.y())
            {
                continue;
            }
            prev = index;
            if (!buffer.inBounds(index))
            {
                continue;
            }

            // use the distance to the center of the cell, since 'proj' can be anywhere in the cell
            Vector3i target_center = index * MAP_RESOLUTION + Vector3i::Constant(MAP_RESOLUTION / 2);
            int value = (point - target_center).norm();
            value = std::min(value, tau);
            if (len > distance)
            {
                value = -value;
            }
            // Calculate the corresponding weight for every TSDF value
            int weight = WEIGHT_RESOLUTION;
            if (value < -weight_epsilon)
            {
                weight = WEIGHT_RESOLUTION * (tau + value) / (tau - weight_epsilon);
            }
            if (weight == 0)
            {
                continue;
            }
            
            // auto object = std::make_pair(value, weight);

            int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;

            int lowest = (proj.z() - delta_z) >> MAP_SHIFT;
            int highest = (proj.z() + delta_z) >> MAP_SHIFT;
            for (index.z() = lowest; index.z() <= highest; index.z()++)
            {
                if (!buffer.inBounds(index))
                {
                    continue;
                }

                // auto existing = values.try_emplace(index, object);
                // if (!existing.second && abs(value) < abs(existing.first->second.first))
                // {
                //     existing.first->second = object;
                // }

                auto& entry = buffer.value(index);
                entry.first = (entry.first * entry.second + value * weight) / (entry.second + weight);
                entry.second = std::min(max_weight, entry.second + weight);
            }
        }
    }

        // wait for all threads to fill their local_values
        //#pragma omp barrier

        // for (auto& map_entry : local_values)
        // {
        //     bool skip = false;
        //     for (int i = 0; i < thread_count; i++)
        //     {
        //         if (i == current_thread)
        //         {
        //             continue;
        //         }
        //         auto iter = values[i].find(map_entry.first);
        //         if (iter != values[i].end() && abs(iter->second.first) < abs(map_entry.second.first))
        //         {
        //             skip = true;
        //             break;
        //         }
        //     }
        //     if (skip)
        //     {
        //         continue;
        //     }
        //     auto& index = map_entry.first;
        //     int value = map_entry.second.first;
        //     int weight = map_entry.second.second;

        //     auto& entry = buffer.value(index.x(), index.y(), index.z());
        //     entry.first = (entry.first * entry.second + value * weight) / (entry.second + weight);
        //     entry.second = std::min(max_weight, entry.second + weight);
        // }
    
}

} // namespace fastsense::tsdf