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
#include <tsdf/krnl_tsdf.h>
#include <registration/registration.h>
#include <util/config/config_manager.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/point_cloud.h>
#include <callback/map_thread.h>

namespace fastsense::callback
{

using Registration = fastsense::registration::Registration;
using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using Eigen::Matrix4f;
using fastsense::util::config::ConfigManager;
using fastsense::buffer::InputBuffer;

class CloudCallback : public fastsense::util::ProcessThread
{
public:
    CloudCallback(Registration& registration,
                  const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                  const std::shared_ptr<LocalMap>& local_map,
                  const std::shared_ptr<GlobalMap>& global_map,
                  const msg::TransformStampedBuffer::Ptr& transform_buffer,
                  const msg::PointCloudPtrStampedBuffer::Ptr& pointcloud_buffer,
                  bool send_after_registration,
                  const fastsense::CommandQueuePtr& q,
                  MapThread& map_thread,
                  std::mutex& map_mutex);

    /**
     * @brief Set the global map
     * 
     * @param global_map the new global map
     */
    void set_global_map(const std::shared_ptr<GlobalMap>& global_map);

    /**
     * @brief Set the local map
     * 
     * @param local_map the new local map
     */
    void set_local_map(const std::shared_ptr<LocalMap>& local_map);

protected:
    /**
     * @brief Contains main functionality of the cloud callback. Mainly gets the current pointcloud out of the buffer and calls the registration and map update step.
     */
    void thread_run() override;

private:
    Registration& registration;

    // pointer to the buffer that holds the pointcloud from the lidar
    std::shared_ptr<PointCloudBuffer> cloud_buffer;

    // pointer to the local map
    std::shared_ptr<LocalMap> local_map;

    // pointer to the global map
    std::shared_ptr<GlobalMap> global_map;

    // 4x4 Matrix that contains the current pose the system inside the map
    Matrix4f pose;

    // pointer to the buffer that the transform object is written into in order to send it to the host pc
    msg::TransformStampedBuffer::Ptr transform_buffer;

    // pointer to the buffer that the pointcloud is written into (after preprocessing and registration) in order to send it to the host pc
    std::shared_ptr<PointCloudBuffer> pointcloud_buffer;

    // boolean that controls whether or not the pointcloud (after preprocessing and registration) should be send to the host pc or not    
    bool send_after_registration;

    // boolean that holds the information whether a pointcloud is the first one being processed or not 
    bool first_iteration;

    // pointer to the commandqueue (needed in order to start registration and tsdf map update on the FPGA)
    fastsense::CommandQueuePtr q;

    MapThread& map_thread;
    std::mutex& map_mutex;
};

}
