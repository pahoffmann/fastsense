/**
 * @file preprocessing.cpp
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 */

#include "preprocessing.h"
#include <util/config/config_manager.h>

#include <unordered_set>

using namespace fastsense::preprocessing;
using fastsense::buffer::InputBuffer;

namespace std
{

// This function creates a unique hash for a Scanpoint in order to use it in the context of an unordered map
template<> struct hash<fastsense::ScanPoint>
{
    std::size_t operator()(fastsense::ScanPoint const& p) const noexcept
    {
        long long v = ((long long)p.x() << 32) ^ ((long long)p.y() << 16) ^ (long long)p.z();
        return std::hash<long long>()(v);
    }
};
}


Preprocessing::Preprocessing(const std::shared_ptr<PointCloudBuffer>& in_buffer,
                             const std::shared_ptr<PointCloudBuffer>& out_buffer,
                             const std::shared_ptr<PointCloudBuffer>& send_buffer,
                             bool send_original,
                             bool send_preprocessed,
                             float scale)
    : QueueBridge{in_buffer, out_buffer, 0, false}, send_buffer(send_buffer), send_original(send_original), send_preprocessed(send_preprocessed), scale(scale)
{}

void Preprocessing::thread_run()
{
    // Get the amount of rings the used lidar provides
    int expected_rings = util::config::ConfigManager::config().lidar.rings();

    fastsense::msg::PointCloudPtrStamped in_cloud;
    
    // This thread runs as long as the system is active
    while (this->running)
    {

        // Wait until a pointcloud is inside the pointcloud buffer
        if (!this->in_->pop_nb(&in_cloud, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        // Send the raw pointcloud to the host pc if send_original == true
        if (send_original)
        {
            send_buffer->push_nb(in_cloud);
        }

        assert(expected_rings == in_cloud.data_->rings_);

        fastsense::msg::PointCloudPtrStamped out_cloud;
        out_cloud.data_ = in_cloud.data_;
        out_cloud.timestamp_ = in_cloud.timestamp_;

        // If scaling was used then the pointcloud needs to be scaled as well
        if (scale != 1.0f)
        {
            for (auto& point : out_cloud.data_->points_)
            {
                point = (point.cast<float>() * scale).cast<ScanPointType>();
            }
        }

        // Apply median filter
        median_filter(out_cloud, 5);

        // Apply reduction filter
        reduction_filter_closest(out_cloud);

        // Write preprocessed pointcloud into the output buffer for the next step
        this->out_->push_nb(out_cloud, true);

        // Send the preprocessed pointcloud to the host pc if send_preprocessed == true
        if (send_preprocessed)
        {
            send_buffer->push_nb(out_cloud);
        }
    }
}

void Preprocessing::reduction_filter_average(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, std::pair<Vector3i, int>> point_map;
    std::pair<Vector3i, int> default_value = std::make_pair(Vector3i::Zero(), 0);

    point_map.reserve(cloud_points.size());

    for (const auto& point : cloud_points)
    {
        // Points with coordinates (0,0,0) should be filtered out
        if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        {
            continue;
        }

        // Calculate a unique coordinate (id) for each voxel
        ScanPoint voxel(
            std::floor((float)point.x() / MAP_RESOLUTION),
            std::floor((float)point.y() / MAP_RESOLUTION),
            std::floor((float)point.z() / MAP_RESOLUTION)
        );

        auto& avg_point = point_map.try_emplace(voxel, default_value).first->second;
        
        // Accumulate coordinates of the points inside one voxel
        avg_point.first += point.cast<int>();
        
        // Count how many points were accumulated in order to calculate the average
        avg_point.second++;
    }

    // The point cloud got smaller so the size of the vector has to be adjusted
    cloud_points.resize(point_map.size());

    // Iterate through accumulated points and calculate the average for every voxel
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter] = (avg_point.second.first / avg_point.second.second).cast<ScanPointType>();
        counter++;
    }
}

void Preprocessing::reduction_filter_closest(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, std::pair<ScanPoint, int>> point_map;
    std::pair<ScanPoint, int> default_value = std::make_pair(ScanPoint::Zero(), MAP_RESOLUTION * 2);

    point_map.reserve(cloud_points.size());

    for (const auto& point : cloud_points)
    {
        // Points with coordinates (0,0,0) should be filtered out
        if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        {
            continue;
        }

        // Calculate a unique coordinate (id) for each voxel
        ScanPoint voxel_center(
            std::floor((float)point.x() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.y() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.z() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2
        );
        
        int distance = (point - voxel_center).norm();

        // Check if the current point is closer to the voxel center than the previous closest point
        auto& closest = point_map.try_emplace(voxel_center, default_value).first->second;
        if (distance < closest.second)
        {
            closest.first = point;
            closest.second = distance;
        }
    }

    // The point cloud got smaller so the size of the vector has to be adjusted
    cloud_points.resize(point_map.size());
    
    // Write the final chosen points from the unordered map into the pointcloud
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter] = avg_point.second.first;
        counter++;
    }
}

void Preprocessing::reduction_filter_voxel_center(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_set<ScanPoint> point_set;

    for (const auto& point : cloud_points)
    {
        // Points with coordinates (0,0,0) should be filtered out
        if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        {
            continue;
        }

        // Calculate a unique coordinate (id) for each voxel
        ScanPoint voxel_center(
            std::floor((float)point.x() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.y() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.z() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2
        );

        point_set.insert(voxel_center);
    }

    // The point cloud got smaller so the size of the vector has to be adjusted
    cloud_points.resize(point_set.size());
    std::copy(point_set.begin(), point_set.end(), cloud_points.begin());
}


void Preprocessing::reduction_filter_random_point(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, ScanPoint> point_map;

    // Shuffle the whole pointcloud so the first point inside a voxel that is looked at is a random one
    std::random_shuffle(cloud_points.begin(), cloud_points.end());

    for (const auto& point : cloud_points)
    {
        // Points with coordinates (0,0,0) should be filtered out
        if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        {
            continue;
        }

        // Calculate a unique coordinate (id) for each voxel
        ScanPoint voxel(
            std::floor((float)point.x() / MAP_RESOLUTION),
            std::floor((float)point.y() / MAP_RESOLUTION),
            std::floor((float)point.z() / MAP_RESOLUTION)
        );

        point_map.try_emplace(voxel, point);
    }

    // The point cloud got smaller so the size of the vector has to be adjusted
    cloud_points.resize(point_map.size());

    // Write the final chosen points from the unordered map into the pointcloud
    int counter = 0;
    for (auto& c_point : point_map)
    {
        cloud_points[counter] = c_point.second;
        counter++;
    }
}


uint8_t Preprocessing::median_from_array(std::vector<ScanPoint*> medians)
{
    // Vector that holds a std::pair<int: "index of the corresponding element of the input vector", double: "euclidian distance of that point to the scanner origin">
    std::vector<std::pair<int, double>> distances(medians.size());
    for (uint8_t i = 0; i < distances.size(); i++)
    {
        distances[i].first = i;
        distances[i].second = medians[i]->norm();
    }

    // Sort the first n/2 elements of the distances vector (you don't need more to determine the median value)
    std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end(), [](auto & left, auto & right)
    {
        return left.second < right.second;
    });

    // Return the index of the median value (not the distance)
    return distances[distances.size() / 2].first;
}

void Preprocessing::median_filter(fastsense::msg::PointCloudPtrStamped& cloud, uint8_t window_size)
{
    // The window size should be equal in order for a point to have an equal amount of neighbour points to the left and to the right
    if (window_size % 2 == 0)
    {
        return;
    }

    std::vector<ScanPoint> result(cloud.data_->points_.size());

    int half_window_size = window_size / 2;

    // Iterate through the pointcloud ringwise
    #pragma omp parallel for schedule(static) shared(result)
    for (uint8_t ring = 0; ring < cloud.data_->rings_; ring++)
    {
        // This vector is used to store the current point that is being looked at plus all the points inside its window
        std::vector<ScanPoint*> window(window_size);

        // Iterate through all the points of the current ring
        for (uint32_t point = 0; point < (cloud.data_->points_.size() / cloud.data_->rings_); point++)
        {
            // Index of the current point that is being looked at inside the pointcloud vector
            int i = (point * cloud.data_->rings_) + ring;

            // Index of the most left neighbour point of the current point (not the index in the pointcloud vector! This value can be negative)
            int first_element = (i - (half_window_size * cloud.data_->rings_));
            
            // Find all points inside the window of the current point and write them into the window vector
            for (uint8_t j = 0; j < window_size; j++)
            {
                // Convert the first_element index into the real index of the point in the pointcloud vector
                int index = ((first_element + (j * cloud.data_->rings_)) + cloud.data_->points_.size()) % cloud.data_->points_.size();
                window[j] = (ScanPoint*)&cloud.data_->points_[index];
            }

            // Calculate the median of the window vector and replace the current point with the median point coordinates
            result[i] = *window[median_from_array(window)];
        }
    }
    
    cloud.data_->points_ = result;
}