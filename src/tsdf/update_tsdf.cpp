
#include <tsdf/update_tsdf.h>

#include <tsdf/ScanOrderNeighbors.h>
#include <tsdf/PriorityNeighbors.h>

#include <tsdf/AverageNormal.h>
#include <tsdf/ProjectionNormal.h>
#include <tsdf/RANSACNormal.h>
#include <tsdf/IterativeNormal.h>

#include <tsdf/weighting.h>

#include <memory>
#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

//#include <omp.h>

/// Weight calculation object
std::unique_ptr<fastsense::tsdf::WeightingBase<float>> weighting_base_(nullptr);

// Normal calculation objects
std::unique_ptr<fastsense::tsdf::NormalBase<Vector3, ScanPoints_t<Vector3>>> normal_base_(nullptr);
std::shared_ptr<fastsense::tsdf::NeighborsBase<Vector3, ScanPoints_t<Vector3>, std::vector<Vector3>>> neighbors_base_(nullptr);

using Point = Eigen::Vector3i;
using Map = std::unordered_map<Point, std::pair<float, float>>;

namespace std
{
template<> struct hash<Point>
{
    std::size_t operator()(Point const& p) const noexcept
    {
        long long v = ((long long)p.x() << 32) ^ ((long long)p.y() << 16) ^ (long long)p.z();
        return std::hash<long long>()(v);
    }
};
}

namespace fastsense::tsdf
{

Point to_point(const Vector3& vec)
{
    return Point(static_cast<int>(std::floor(vec.x())),
                 static_cast<int>(std::floor(vec.y())),
                 static_cast<int>(std::floor(vec.z())));
}

void update_tsdf(const ScanPoints_t<Vector3>& scan_points,
                 const Vector3& scanner_pos,
                 fastsense::map::LocalMap<std::pair<float, float>>& buffer,
                 TsdfCalculation tsdf_method,
                 float tau,
                 float max_weight)
{
    bool interpolate = tsdf_method >= 10;
    tsdf_method = (TsdfCalculation)(tsdf_method % 10);

    if (weighting_base_.get() == nullptr)
    {
        weighting_base_.reset(new LinearWeight<float>(tau, tau * 0.1));
        neighbors_base_.reset(new ScanOrderNeighbors<Vector3, ScanPoints_t<Vector3>>(1, 20));
        switch (tsdf_method)
        {
        case TsdfCalculation::AVERAGE:
            normal_base_.reset(new AverageNormal<Vector3, ScanPoints_t<Vector3>>(20,  tau * tau / 4));
            break;
        case TsdfCalculation::RANSAC:
            normal_base_.reset(new RANSACNormal<Vector3, ScanPoints_t<Vector3>>(neighbors_base_));
            break;
        case TsdfCalculation::ITERATIVE:
            normal_base_.reset(new IterativeNormal<Vector3, ScanPoints_t<Vector3>>(neighbors_base_));
            break;
        default:
            break;
        }
    }

    const auto& calc_weight = *weighting_base_;
    size_t width = scan_points[0].size();

    float angle = 30.0 / scan_points.size(); // TODO: Scanner FoV as Parameter
    float dz_per_distance = std::tan(angle / 180 * M_PI) / 2.0;

    //int thread_count = omp_get_max_threads();
    int thread_count = 1;

    std::vector<Map> values(thread_count);

    //#pragma omp parallel num_threads(thread_count)
    {
        //int current_thread = omp_get_thread_num();
        int current_thread = 0;
        Map& local_values = values[current_thread];
        std::unordered_set<Point> visited;

        //#pragma omp for schedule(static)
        for (size_t i = 0; i < width; i++)
        {
            for (size_t ring = 0; ring < scan_points.size(); ring++)
            {
                const Vector3& point = scan_points[ring][i];

                // Skip if point is invalid
                if (std::isnan(point.x()))
                {
                    continue;
                }

                Vector3 direction_vector = point - scanner_pos;
                float distance = direction_vector.norm();
                direction_vector /= distance;

                if (tsdf_method != TsdfCalculation::PROJECTION)
                {
                    Vector3 normal;

                    // Try to calculate the normal of the current point
                    try
                    {
                        normal = (*normal_base_)(ring, i, scan_points);
                    }
                    catch (int)
                    {
                        continue;
                    }

                    // measure how good the normal can be
                    float accuracy = -normal.dot(direction_vector);

                    // discard normals perpendicular to the scanner
                    if (accuracy == 0)
                    {
                        continue;
                    }
                    // flip normals that point away from the scanner
                    if (accuracy < 0)
                    {
                        normal = -normal;
                        accuracy = -accuracy;
                    }

                    float delta_ab = 0;
                    Vector3 axis_a = Vector3::Zero();
                    Vector3 axis_b = Vector3::Zero();
                    if (interpolate)
                    {
                        // construct a, b, normal such that a _|_ b _|_ normal _|_ a
                        // aka construct a 3D base in normal direction
                        axis_a = normal.cross(Vector3::UnitY());
                        if (axis_a == Vector3::Zero())
                        {
                            axis_a = normal.cross(Vector3::UnitX());
                        }
                        axis_b = normal.cross(axis_a);
                        axis_a = normal.cross(axis_b);
                        delta_ab = dz_per_distance * distance * 2;
                    }

                    visited.clear();

                    for (float len_a = -delta_ab; len_a <= delta_ab; len_a += 0.5)
                    {
                        for (float len_b = -delta_ab; len_b <= delta_ab; len_b += 0.5)
                        {
                            for (float len_n = -tau; len_n <= tau; len_n += 0.5)
                            {
                                Vector3 proj = point + normal * len_n
                                               + axis_a * len_a + axis_b * len_b;
                                Point index = to_point(proj);
                                if (visited.find(index) != visited.end())
                                {
                                    continue;
                                }
                                visited.insert(index);
                                if (!buffer.inBounds(index.x(), index.y(), index.z()))
                                {
                                    continue;
                                }

                                // use the distance to the center of the cell, since 'proj' can be anywhere in the cell
                                Vector3 target_center = index.cast<float>() + Vector3::Constant(0.5);
                                Vector3 delta = point - target_center;
                                float value = delta.norm();
                                if (value > tau)
                                {
                                    continue;
                                }
                                if (normal.dot(delta) > 0)
                                {
                                    value = -value;
                                }
                                // Calculate the corresponding weight for every TSDF value
                                float weight = calc_weight(-value) * accuracy;
                                if (weight == 0)
                                {
                                    continue;
                                }
                                auto object = std::make_pair(value, weight);
                                auto existing = local_values.try_emplace(index, object);
                                if (!existing.second && fabsf(value) < fabsf(existing.first->second.first))
                                {
                                    existing.first->second = object;
                                }
                            }
                        }
                    }
                }
                else
                {
                    Point prev(0, 0, 0);

                    for (float len = 1; len <= distance + tau; len += 0.5)
                    {
                        Vector3 proj = scanner_pos + direction_vector * len;
                        Point index = to_point(proj);
                        if (index.x() == prev.x() && index.y() == prev.y())
                        {
                            continue;
                        }
                        prev = index;
                        if (!buffer.inBounds(index.x(), index.y(), index.z()))
                        {
                            continue;
                        }

                        // use the distance to the center of the cell, since 'proj' can be anywhere in the cell
                        Vector3 target_center = index.cast<float>() + Vector3::Constant(0.5);
                        float value = (point - target_center).norm();

                        value = std::min(value, tau);

                        if (len > distance)
                        {
                            value = -value;
                        }
                        // Calculate the corresponding weight for every TSDF value
                        float weight = calc_weight(-value);
                        if (weight == 0)
                        {
                            continue;
                        }
                        auto object = std::make_pair(value, weight);

                        float delta_z = 0;
                        if (interpolate)
                        {
                            delta_z = dz_per_distance * len;
                        }

                        int lowest = (int)floor(proj.z() - delta_z);
                        int highest = (int)floor(proj.z() + delta_z);
                        for (index.z() = lowest; index.z() <= highest; index.z()++)
                        {
                            if (!buffer.inBounds(index.x(), index.y(), index.z()))
                            {
                                continue;
                            }

                            auto existing = local_values.try_emplace(index, object);
                            if (!existing.second && fabsf(value) < fabsf(existing.first->second.first))
                            {
                                existing.first->second = object;
                            }
                        }
                    }
                }
            }
        }

        // wait for all threads to fill their local_values
        //#pragma omp barrier

        for (auto& map_entry : local_values)
        {
            bool skip = false;
            for (int i = 0; i < thread_count; i++)
            {
                if (i == current_thread)
                {
                    continue;
                }
                auto iter = values[i].find(map_entry.first);
                if (iter != values[i].end() && fabsf(iter->second.first) < fabsf(map_entry.second.first))
                {
                    skip = true;
                    break;
                }
            }
            if (skip)
            {
                continue;
            }
            auto& index = map_entry.first;
            float value = map_entry.second.first;
            float weight = map_entry.second.second;

            auto& entry = buffer.value(index.x(), index.y(), index.z());
            entry.first = (entry.first * entry.second + value * weight) / (entry.second + weight);
            entry.second = std::min(max_weight, entry.second + weight);
        }
    }
}

} // namespace fastsense::tsdf