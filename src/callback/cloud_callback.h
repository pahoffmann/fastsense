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
#include <util/config/config_manager.h>
#include <tsdf/update_tsdf.h>
#include <msg/tsdf_bridge_msg.h>

namespace fastsense::callback
{

    using Registration = fastsense::registration::Registration;
    using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>;
    using fastsense::map::LocalMap;
    using fastsense::map::GlobalMap;
    using Eigen::Matrix4f;
    using fastsense::util::config::ConfigManager;
    using TSDFBuffer = util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>;
    using fastsense::buffer::InputBuffer;

    class CloudCallback : public fastsense::util::ProcessThread{
        public:
            CloudCallback(Registration& registration, const std::shared_ptr<PointCloudBuffer>& cloud_buffer, LocalMap& local_map, const std::shared_ptr<GlobalMap>& global_map, Matrix4f& pose, const std::shared_ptr<TSDFBuffer>& tsdf_buffer, fastsense::CommandQueuePtr& q);

            void start() override;

            void callback();

            void preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, InputBuffer<Point>& scan_point);

            int determineBufferSize(const fastsense::msg::PointCloudStamped& cloud);

            void stop() override;

        private:
            Registration& registration;
            std::shared_ptr<PointCloudBuffer> cloud_buffer;
            LocalMap& local_map;
            std::shared_ptr<GlobalMap> global_map;
            Matrix4f& pose;
            std::shared_ptr<TSDFBuffer> tsdf_buffer;
            fastsense::CommandQueuePtr& q;

            bool first_iteration;
    };
}