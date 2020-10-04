#pragma once

/**
 * 
 * @author Pascal Buschermöhle
 * 
 */

#include <util/process_thread.h> 
#include <registration/registration.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/msgs_stamped.h>
#include <map/local_map.h>
#include <eigen3/Eigen/Dense>

namespace fastsense::callback
{

    using Registration = fastsense::registration::Registration;
    using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>;
    using fastsense::map::LocalMap;
    using fastsense::map::GlobalMap;
    using Eigen::Matrix4f;

    class CloudCallback : public fastsense::util::ProcessThread{
        public:
            CloudCallback(Registration& registration, std::shared_ptr<PointCloudBuffer>& cloud_buffer, LocalMap& local_map, std::shared_ptr<GlobalMap>& global_map, Matrix4f& pose);

            void start() override;

            void callback();

            void preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, ScanPoints_t& scan_points, float map_resolution, const Matrix4f& pose);

            void stop() override;

        private:
            Registration& registration;
            std::shared_ptr<PointCloudBuffer>& cloud_buffer;
            LocalMap& local_map;
            std::shared_ptr<GlobalMap>& global_map;
            Matrix4f& pose;
    };
}