/**
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <unordered_map>

#include <tsdf/update_tsdf_hw.h>
#include <tsdf/weighting.h>
#include <util/hls_functions.h>

namespace std
{
template<> struct hash<PointHW>
{
    std::size_t operator()(PointHW const& p) const noexcept
    {
        long long v = ((long long)p.x << 32) ^ ((long long)p.y << 16) ^ (long long)p.z;
        return std::hash<long long>()(v);
    }
};
}

namespace fastsense::tsdf
{

void update_tsdf_hw(const fastsense::buffer::InputBuffer<PointHW>& scan_points,
                 fastsense::map::LocalMap& buffer,
                 int tau,
                 int max_weight)
{
    int weight_epsilon = tau / 10;

    std::unordered_map<PointHW, std::pair<int, int>> values;

    PointHW scanner_pos(buffer.get_pos().x(), buffer.get_pos().y(), buffer.get_pos().z());

    for (size_t i = 0; i < scan_points.size(); i++)
    {
        const auto& point = scan_points[i];


        auto direction_vector = point - scanner_pos;
        int distance = direction_vector.norm();

        PointHW prev(0, 0, 0);
        for (int len = MAP_RESOLUTION; len <= distance + tau; len += MAP_RESOLUTION / 2)
        {
            auto proj = scanner_pos + direction_vector * len / distance;
            
            auto index = proj / MAP_RESOLUTION;
 
            if (index.x == prev.x && index.y == prev.y)
            {
                continue;
            }
            prev = index;
            if (!buffer.in_bounds(index.x, index.y, index.z))
            {
                continue;
            }

            // use the distance to the center of the cell, since 'proj' can be anywhere in the cell
            auto target_center = index * MAP_RESOLUTION + PointHW(MAP_RESOLUTION / 2, MAP_RESOLUTION / 2, MAP_RESOLUTION / 2); //Vector3i::Constant(MAP_RESOLUTION / 2);
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
            
            auto object = std::make_pair(value, weight);

            int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;

            int lowest = (proj.z - delta_z) / MAP_RESOLUTION;
            int highest = (proj.z + delta_z) / MAP_RESOLUTION;

            for (index.z = lowest; index.z <= highest; index.z++)
            {
                if (!buffer.in_bounds(index.x, index.y, index.z))
                {
                    continue;
                }

                auto existing = values.try_emplace(index, object);
                if (!existing.second && abs(value) < abs(existing.first->second.first))
                {
                    existing.first->second = object;
                }
            }
        }
    }

    // wait for all threads to fill their local_values

    for (auto& map_entry : values)
    {
        auto& index = map_entry.first;
        int value = map_entry.second.first;
        int weight = map_entry.second.second;

        auto& entry = buffer.value(index.x, index.y, index.z);
        entry.first = (entry.first * entry.second + value * weight) / (entry.second + weight);
        entry.second = std::min(max_weight, entry.second + weight);
    }
}

} // namespace fastsense::tsdf
