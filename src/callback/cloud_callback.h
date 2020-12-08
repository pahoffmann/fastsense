#pragma once

/**
 * @file cloud_callback.h
 * @author Pascal Buschermöhle
 */

#include <util/point_hw.h>
#include <msg/transform.h>
#include <map/local_map.h>
#include <eigen3/Eigen/Dense>
#include <util/process_thread.h>
#include <msg/tsdf_bridge_msg.h>
#include <hw/kernels/tsdf_kernel.h>
#include <registration/registration.h>
#include <util/config/config_manager.h>
#include <util/concurrent_ring_buffer.h>
#include <preprocessing/preprocessing.h>
#include <callback/map_thread.h>

namespace fastsense::callback
{

using Registration = fastsense::registration::Registration;
using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using Eigen::Matrix4f;
using fastsense::util::config::ConfigManager;
using fastsense::buffer::InputBuffer;
using TransformBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::Transform>;

class CloudCallback : public fastsense::util::ProcessThread
{
public:
    CloudCallback(Registration& registration,
                  const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                  const std::shared_ptr<LocalMap>& local_map,
                  const std::shared_ptr<GlobalMap>& global_map,
                  Matrix4f& pose,
                  const std::shared_ptr<TransformBuffer>& transform_buffer,
                  fastsense::CommandQueuePtr& q,
                  MapThread& map_thread,
                  std::mutex& map_mutex);

protected:
    void thread_run() override;

private:
    Registration& registration;
    std::shared_ptr<PointCloudBuffer> cloud_buffer;
    std::shared_ptr<LocalMap> local_map;
    std::shared_ptr<GlobalMap> global_map;
    Matrix4f& pose;
    std::shared_ptr<TransformBuffer> transform_buffer;
    bool first_iteration;
    fastsense::CommandQueuePtr& q;
    fastsense::kernels::TSDFKernel tsdf_krnl;
    fastsense::preprocessing::Preprocessing preprocessor;
    MapThread& map_thread;
    std::mutex& map_mutex;
};

}
