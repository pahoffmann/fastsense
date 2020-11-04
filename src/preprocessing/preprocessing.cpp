#include "preprocessing.h"

using namespace fastsense::preprocessing;
using fastsense::msg::Point;
using fastsense::buffer::InputBuffer;

struct AveragePoint
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t count;

    AveragePoint()
    {
        x = 0;
        y = 0;
        z = 0;
        count = 0;
    }
};

void Preprocessing::preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, InputBuffer<PointHW>& scan_points, const Matrix4f& pose)
{
    const std::vector<fastsense::msg::Point>& cloud_points = cloud.first->points_;
    Eigen::Vector4f v;

    int scan_points_index = 0;
    for (unsigned int i = 0; i < cloud_points.size(); i++)
    {
        v << cloud_points[i].x, cloud_points[i].y, cloud_points[i].z, 1.0f;

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
    std::vector<fastsense::msg::Point>& cloud_points = cloud.first->points_;

    std::unordered_map<uint64_t, AveragePoint> point_map;

    point_map.reserve(cloud_points.size());
    for (uint32_t i = 0; i < cloud_points.size(); i++)
    {
        if (cloud_points[i].x == 0 && cloud_points[i].y == 0 && cloud_points[i].z == 0)
        {
            continue;
        }

        uint64_t key = 0;
        int16_t* key_ptr = (int16_t*)&key;
        key_ptr[0] = (int16_t)std::floor((cloud_points[i].x) / MAP_RESOLUTION);
        key_ptr[1] = (int16_t)std::floor((cloud_points[i].y) / MAP_RESOLUTION);
        key_ptr[2] = (int16_t)std::floor((cloud_points[i].z) / MAP_RESOLUTION);

        AveragePoint& avg_point = point_map[key];
        avg_point.count++;
        avg_point.x += cloud_points[i].x;
        avg_point.y += cloud_points[i].y;
        avg_point.z += cloud_points[i].z;
    }

    cloud_points.resize(point_map.size());
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter].x = avg_point.second.x / avg_point.second.count;
        cloud_points[counter].y = avg_point.second.y / avg_point.second.count;
        cloud_points[counter].z = avg_point.second.z / avg_point.second.count;
        counter++;
    }
}

uint8_t Preprocessing::median_from_array(std::vector<Point*> medians){
    std::vector<std::pair<int, double>> distances(medians.size());
    
    for(uint8_t i = 0; i < distances.size(); i++){
        distances[i].first = i;
        distances[i].second = sqrt((medians[i]->x * medians[i]->x) + (medians[i]->y * medians[i]->y) + (medians[i]->z * medians[i]->z));
    }

    std::sort(distances.begin(), distances.end(), [](auto &left, auto &right) {
    return left.second < right.second;
    });


     return distances[(int)std::floor(medians.size()/2)].first;
}


template <typename T>
T shift_array_by_one(std::vector<T> array){
    T ret = array[0];
    for(uint8_t i = 0; i < array.size()-1; i++){
        array[i] = array[i + 1];
    }

    return ret;
}

void Preprocessing::median_filter(fastsense::msg::PointCloudStamped& cloud, uint8_t window_size){
    if(window_size % 2 == 0) return;

    int half_window_size = (int)std::ceil(window_size/2.0f);

    std::vector<std::vector<Point*>> window(cloud.first->rings_, std::vector<Point*>(window_size, nullptr));
    std::vector<std::vector<Point>> medians(cloud.first->rings_, std::vector<Point>(half_window_size, {0, 0, 0}));

    uint16_t current_ring;
    for(uint16_t i = 0; i < cloud.first->points_.size(); i++){
        current_ring = i % cloud.first->rings_;
        window[current_ring][window_size-1] = &cloud.first->points_[i];
        
        if(window[current_ring][0] != nullptr){
            Point* element = window[current_ring][median_from_array(window[current_ring])];
            medians[current_ring][half_window_size-1] = {element->x, element->y, element->z};
        }

        Point* old_value = shift_array_by_one(window[current_ring]);
        Point new_value = shift_array_by_one(medians[current_ring]);

        if(new_value.x == 0 && new_value.y == 0 && new_value.z == 0) continue;
        old_value->x = new_value.x;
        old_value->y = new_value.y;
        old_value->z = new_value.z;
    }
}