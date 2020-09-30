/**
 * @file 
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <omp.h>
#include <ros/ros.h>
#include <bridge/tsdf_bridge.h>

using namespace fastsense::bridge;

TSDFBridge::TSDFBridge(ros::NodeHandle& n) 
:   BridgeBase{n, "/tsdf_bridge/tsdf"}, 
    ProcessThread{},
    points_{},
    colors_{}
{
    points_.reserve(30'000);
    colors_.reserve(30'000);
}

void TSDFBridge::start()
{
    if (running == false)
    {
        running = true;
        worker = std::thread(&TSDFBridge::run, this);
    }
}

void TSDFBridge::stop()
{
    running = false;
    worker.join();
}

void TSDFBridge::run()
{
    while (running && ros::ok())
    {
        BridgeBase::run();
    }
}

// TODO in util, so steffen and my version match
bool TSDFBridge::inBounds(int x, int y, int z)
{
    return abs(x - msg().pos_.x()) <= msg().size_.x() / 2 && abs(y - msg().pos_.y()) <= msg().size_.y() / 2 && abs(z - msg().pos_.z()) <= msg().size_.z() / 2;
}

std::pair<float, float> TSDFBridge::get_tsdf_value(int x, int y, int z)
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    
    return msg().tsdf_data_[((x - msg().pos_.x() + msg().offset_.x() + msg().size_.x()) % msg().size_.x()) * msg().size_.y() * msg().size_.z() +
                     ((y - msg().pos_.y() + msg().offset_.y() + msg().size_.y()) % msg().size_.y()) * msg().size_.z() +
                     (z - msg().pos_.z() + msg().offset_.z() + msg().size_.z()) % msg().size_.z()];
}

void TSDFBridge::convert()
{   
    constexpr size_t thread_count = 20;
    std::vector<std::pair<geometry_msgs::Point, std_msgs::ColorRGBA>> results[thread_count];

    int left[3], right[3];

    for (int i = 0; i < 3; i++)
    {
        left[i] = msg().pos_.at(i) - msg().size_.at(i) / 2;
        right[i] = msg().pos_.at(i) + msg().size_.at(i) / 2;
    }

    #pragma omp parallel num_threads(thread_count)
    {
        auto& result = results[omp_get_thread_num()];
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = 0;

        #pragma omp for collapse(3) schedule(static)
        for (int x = left[0]; x <= right[0]; x++)
        {
            for (int y = left[1]; y <= right[1]; y++)
            {
                for (int z = left[2]; z <= right[2]; z++)
                {
                    auto val = get_tsdf_value(x, y, z);
                    if (val.second == 0 || fabsf(val.first) >= msg().tau_)
                    {
                        continue;
                    }

                    geometry_msgs::Point point;
                    point.x = x * msg().map_resolution_;
                    point.y = y * msg().map_resolution_;
                    point.z = z * msg().map_resolution_;

                    // color.a = std::min(val.second, 1.0f);
                    if (val.first >= 0)
                    {
                        color.r = val.first / msg().tau_;
                        color.g = 0;
                    }
                    else
                    {
                        color.r = 0;
                        color.g = -val.first / msg().tau_;
                    }

                    result.push_back(std::make_pair(point, color));
                }
            }
        }
    }

    std::vector<int> offsets(thread_count, 0);
    size_t total_results = 0;
    for (int i = 0; i < thread_count; i++)
    {
        offsets[i] = total_results;
        total_results += results[i].size();
    }
    
    points_.clear();
    colors_.clear();

    if (total_results == 0)
    {
        return;
    }

    #pragma omp parallel num_threads(thread_count)
    {
        auto& result = results[omp_get_thread_num()];
        int offset = offsets[omp_get_thread_num()];
        for (int i = 0; i < result.size(); i++)
        {
            auto& p = result[i];
            points_[i + offset] = p.first;
            colors_[i + offset] = p.second;
        }
    }
}

void TSDFBridge::publish()
{
    visualization_msgs::Marker marker;
    marker.header = std_msgs::Header{};
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "window";
    marker.id = 0;
    marker.scale.x = marker.scale.y = msg().map_resolution_ * 0.6;
    marker.points = points_;
    marker.colors = colors_;
    pub().publish(marker);
}