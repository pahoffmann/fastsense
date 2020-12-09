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

namespace fastsense::callback
{

using Registration = fastsense::registration::Registration;
using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using Eigen::Matrix4f;
using fastsense::util::config::ConfigManager;
using TSDFBuffer = util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>;
using fastsense::buffer::InputBuffer;

class VisPublisher : public fastsense::util::ProcessThread
{
public:
    using BufferPtr = std::shared_ptr<util::ConcurrentRingBuffer<Matrix4f>>;

    VisPublisher(const BufferPtr& buffer, const std::shared_ptr<LocalMap>& local_map, const std::shared_ptr<TSDFBuffer>& tsdf_buffer)
        : vis_buffer{buffer}, local_map{local_map}, tsdf_buffer{tsdf_buffer}
    {}
protected:
    void thread_run() override;
private:
    BufferPtr vis_buffer;
    std::shared_ptr<LocalMap> local_map;
    std::shared_ptr<TSDFBuffer> tsdf_buffer;
};

class CloudCallback : public fastsense::util::ProcessThread
{
public:
    CloudCallback(Registration& registration,
                  const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                  const std::shared_ptr<LocalMap>& local_map,
                  const std::shared_ptr<GlobalMap>& global_map,
                  Matrix4f& pose,
                  const VisPublisher::BufferPtr& vis_buffer,
                  const msg::TransformStampedBuffer::Ptr& transform_buffer,
                  fastsense::CommandQueuePtr& q);

protected:
    void thread_run() override;

private:
    Registration& registration;
    std::shared_ptr<PointCloudBuffer> cloud_buffer;
    std::shared_ptr<LocalMap> local_map;
    std::shared_ptr<GlobalMap> global_map;
    Matrix4f& pose;
    msg::TransformStampedBuffer::Ptr transform_buffer;
    bool first_iteration;
    fastsense::CommandQueuePtr& q;
    fastsense::kernels::TSDFKernel tsdf_krnl;
    fastsense::preprocessing::Preprocessing preprocessor;
    VisPublisher::BufferPtr vis_buffer;
};
}
