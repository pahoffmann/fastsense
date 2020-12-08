#pragma once 

/**
 * @file map_thread.h
 * @author Steffen Hinderink, Marc Eisoldt
 */

#include <util/point_hw.h>
#include <msg/transform.h>
#include <map/local_map.h>
#include <eigen3/Eigen/Dense>
#include <util/process_thread.h>
#include <msg/tsdf_bridge_msg.h>
#include <hw/kernels/tsdf_kernel.h>
#include <util/config/config_manager.h>
#include <util/concurrent_ring_buffer.h>
#include <mutex>
#include <atomic>

namespace fastsense::callback
{

using Eigen::Vector3i;
using TSDFBuffer = util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>;

class MapThread : public fastsense::util::ProcessThread
{
public:

    MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map, 
              std::mutex& map_mutex,
              std::shared_ptr<TSDFBuffer> tsdf_buffer,
              fastsense::CommandQueuePtr& q);

    void go(const Vector3i& pos, const fastsense::buffer::InputBuffer<PointHW>& points);

    void stop() override;

protected:
    void thread_run() override;

private:
    const std::shared_ptr<fastsense::map::LocalMap> local_map_;
    fastsense::kernels::TSDFKernel tsdf_krnl_;
    std::mutex& map_mutex_;
    std::mutex start_mutex_;
    std::atomic<bool> active_;
    Vector3i pos_;
    std::unique_ptr<fastsense::buffer::InputBuffer<PointHW>> points_ptr_;
    int reg_cnt_;
    std::shared_ptr<TSDFBuffer> tsdf_buffer_;
};

} // namespace fastsense::callback
