#include "preprocessing.h"

using namespace fastsense::preprocessing;
using fastsense::buffer::InputBuffer;

void Preprocessing::preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, InputBuffer<PointHW>& scan_points, const Matrix4f& pose)
{
    const auto& cloud_points = cloud.first->points_;
    Eigen::Vector4f v;

    int scan_points_index = 0;
    for (unsigned int i = 0; i < cloud_points.size(); i++)
    {
        v << cloud_points[i].cast<float>(), 1.0f;

        Vector3f pointf = (pose * v).block<3, 1>(0, 0);
        PointHW point{static_cast<int>(std::floor(pointf.x())),
                      static_cast<int>(std::floor(pointf.y())),
                      static_cast<int>(std::floor(pointf.z()))};

        scan_points[scan_points_index] = point;
        scan_points_index++;

    }
}

void Preprocessing::reduction_filter(fastsense::msg::PointCloudStamped& cloud)
{
    auto& cloud_points = cloud.first->points_;

    std::unordered_map<uint64_t, std::pair<Vector3i, int>> point_map;
    std::pair<Vector3i, int> default_value = std::make_pair(Vector3i::Zero(), 0);

    point_map.reserve(cloud_points.size());
    for (uint32_t i = 0; i < cloud_points.size(); i++)
    {
        if (cloud_points[i].x() == 0 && cloud_points[i].y() == 0 && cloud_points[i].z() == 0)
        {
            continue;
        }

        uint64_t key = 0;
        int16_t* key_ptr = (int16_t*)&key;
        key_ptr[0] = cloud_points[i].x() / MAP_RESOLUTION;
        key_ptr[1] = cloud_points[i].y() / MAP_RESOLUTION;
        key_ptr[2] = cloud_points[i].z() / MAP_RESOLUTION;

        auto& avg_point = point_map.try_emplace(key, default_value).first->second;
        avg_point.first += cloud_points[i].cast<int>();
        avg_point.second++;
    }

    cloud_points.resize(point_map.size());
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter] = (avg_point.second.first / avg_point.second.second).cast<ScanPointType>();
        counter++;
    }
}

uint8_t Preprocessing::median_from_array(std::vector<ScanPoint*> medians)
{
    std::vector<std::pair<int, double>> distances(medians.size());

    for (uint8_t i = 0; i < distances.size(); i++)
    {
        distances[i].first = i;
        distances[i].second = medians[i]->norm();
    }

    std::sort(distances.begin(), distances.end(), [](auto & left, auto & right)
    {
        return left.second < right.second;
    });


    return distances[(int)std::floor(medians.size() / 2)].first;
}


template <typename T>
T shift_array_by_one(std::vector<T> array)
{
    T ret = array[0];
    for (uint8_t i = 0; i < array.size() - 1; i++)
    {
        array[i] = array[i + 1];
    }

    return ret;
}

void Preprocessing::median_filter(fastsense::msg::PointCloudStamped& cloud, uint8_t window_size)
{
    if (window_size % 2 == 0)
    {
        return;
    }

    int half_window_size = (int)std::ceil(window_size / 2.0f);

    std::vector<std::vector<ScanPoint*>> window(cloud.first->rings_, std::vector<ScanPoint*>(window_size, nullptr));
    std::vector<std::vector<ScanPoint>> medians(cloud.first->rings_, std::vector<ScanPoint>(half_window_size, {0, 0, 0}));

    uint16_t current_ring;
    for (uint16_t i = 0; i < cloud.first->points_.size(); i++)
    {
        current_ring = i % cloud.first->rings_;
        window[current_ring][window_size - 1] = &cloud.first->points_[i];

        if (window[current_ring][0] != nullptr)
        {
            ScanPoint* element = window[current_ring][median_from_array(window[current_ring])];
            medians[current_ring][half_window_size - 1] = {element->x(), element->y(), element->z()};
        }

        ScanPoint* old_value = shift_array_by_one(window[current_ring]);
        ScanPoint new_value = shift_array_by_one(medians[current_ring]);

        if (new_value.x() == 0 && new_value.y() == 0 && new_value.z() == 0)
        {
            continue;
        }
        *old_value = new_value;

    }
}