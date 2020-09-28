/**
 * @file sender.tcc
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <omp.h>
#include <vector>
#include <zmq.hpp>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// #include <map/local_map.h>
#include <msg/pose.h>
#include <util/xyz_buffer.h>
#include <comm/receiver.h>
#include <comm/bridge_messages.h>

namespace fs = fastsense;
using fs::msg::Point;
using fs::comm::Receiver;

using LocalMapSize = fs::util::XYZBuffer<int>;
using LocalMapPos = fs::util::XYZBuffer<int>;
using LocalMapOffset = fs::util::XYZBuffer<int>;


bool inBounds(int x, int y, int z, const fs::util::LocalMapPos& pos, const fs::util::LocalMapSize& size)
{
    return abs(x - pos.x()) <= size.x() / 2 && abs(y - pos.y()) <= size.y() / 2 && abs(z - pos.z()) <= size.z() / 2;
}

// TODO in util, so steffen and my 
std::pair<float, float> get_tsdf_value(std::vector<std::pair<float, float>>& tsdf_data, int x, int y, int z, const fs::util::LocalMapPos& pos, const fs::util::LocalMapSize& size, const fs::util::LocalMapOffset& offset)
{
    if (!inBounds(x, y, z, pos, size))
    {
        throw std::out_of_range("Index out of bounds");
    }
    
    return tsdf_data[((x - pos.x() + offset.x() + size.x()) % size.x()) * size.y() * size.z() +
                     ((y - pos.y() + offset.y() + size.y()) % size.y()) * size.z() +
                     (z - pos.z() + offset.z() + size.z()) % size.z()];
}

// TODO skalierung
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsdf_bridge");
    ros::NodeHandle n;

    Receiver<fs::comm::TSDFBridgeMessage> receiver(5555);
    constexpr size_t thread_count = 12;

    auto marker_pub = n.advertise<visualization_msgs::Marker>("visu_marker", 1);

    while (ros::ok())
    {
        fs::comm::TSDFBridgeMessage msg;
        
        if (auto _nbytes = receiver.receive(msg))
        {
            std::vector<std::pair<geometry_msgs::Point, std_msgs::ColorRGBA>> results[thread_count];

            int left[3], right[3];

            const LocalMapSize& size = msg.size_;
            const LocalMapPos& pos = msg.pos_;
            const LocalMapOffset& offset = msg.offset_;

            for (int i = 0; i < 3; i++)
            {
                left[i] = pos.at(i) - size.at(i) / 2;
                right[i] = pos.at(i) + size.at(i) / 2;
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
                            auto val = get_tsdf_value(msg.tsdf_data_, x, y, z, pos, size, offset);
                            if (val.second == 0 || fabsf(val.first) >= msg.tau_)
                            {
                                continue;
                            }

                            geometry_msgs::Point point;
                            point.x = x * msg.map_resolution_;
                            point.y = y * msg.map_resolution_;
                            point.z = z * msg.map_resolution_;

                            // color.a = std::min(val.second, 1.0f);
                            if (val.first >= 0)
                            {
                                color.r = val.first / msg.tau_;
                                color.g = 0;
                            }
                            else
                            {
                                color.r = 0;
                                color.g = -val.first / msg.tau_;
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
            
            if (total_results == 0)
            {
                continue;
            }

            std::vector<geometry_msgs::Point> points(total_results);
            std::vector<std_msgs::ColorRGBA> colors(total_results);

            #pragma omp parallel num_threads(thread_count)
            {
                auto& result = results[omp_get_thread_num()];
                int offset = offsets[omp_get_thread_num()];
                for (int i = 0; i < result.size(); i++)
                {
                    auto& p = result[i];
                    points[i + offset] = p.first;
                    colors[i + offset] = p.second;
                }
            }

            visualization_msgs::Marker marker;
            marker.header = std_msgs::Header{};
            marker.header.stamp = ros::Time();
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "window";
            marker.id = 0;
            marker.scale.x = marker.scale.y = msg.map_resolution_ * 0.6;
            marker.points = points;
            marker.colors = colors;
            marker_pub.publish(marker);
        }

        ros::spinOnce();
    }

    return 0;
}
