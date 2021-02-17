#include "preprocessing.h"

using namespace fastsense::preprocessing;
using fastsense::buffer::InputBuffer;

void Preprocessing::preprocess_scan(const fastsense::msg::PointCloudPtrStamped& cloud, InputBuffer<PointHW>& scan_points)
{
    const auto& cloud_points = cloud.data_->points_;

    for (unsigned int i = 0; i < cloud_points.size(); i++)
    {
        PointHW point(cloud_points[i].x(),
                      cloud_points[i].y(),
                      cloud_points[i].z());

        scan_points[i] = point;
    }
}

void Preprocessing::reduction_filter(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

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
        key_ptr[0] = std::floor(((float)cloud_points[i].x()) / (float)MAP_RESOLUTION);
        key_ptr[1] = std::floor(((float)cloud_points[i].y()) / (float)MAP_RESOLUTION);
        key_ptr[2] = std::floor(((float)cloud_points[i].z()) / (float)MAP_RESOLUTION);
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

    std::nth_element(distances.begin(), distances.begin() + distances.size()/2, distances.end(), [](auto & left, auto & right)
    {
        return left.second < right.second;
    });

    return distances[distances.size()/2].first;
}

void Preprocessing::median_filter(fastsense::msg::PointCloudPtrStamped& cloud, uint8_t window_size)
{
    if (window_size % 2 == 0)
    {
        return;
    }

    std::vector<ScanPoint> result(cloud.data_->points_.size());
    
    int half_window_size = window_size/2;

    #pragma omp parallel for schedule(static) shared(result)
    for(uint8_t ring = 0; ring < cloud.data_->rings_; ring++)
    {
        std::vector<ScanPoint*> window(window_size);
        for(uint32_t point = 0; point < (cloud.data_->points_.size() / cloud.data_->rings_); point++)
        {
            int i = (point * cloud.data_->rings_) + ring;
            int first_element = (i - (half_window_size * cloud.data_->rings_));
            for(uint8_t j = 0; j < window_size; j++)
            { 
                int index = ((first_element + (j * cloud.data_->rings_)) + cloud.data_->points_.size()) % cloud.data_->points_.size();
                window[j] = (ScanPoint*)&cloud.data_->points_[index];
            }
            
            result[i] = *window[median_from_array(window)];
            
        }
    }

    cloud.data_->points_ = result;
}