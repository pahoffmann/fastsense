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

#include <mutex>

namespace fastsense::callback
{

using Eigen::Vector3i;

class MapThread : public fastsense::util::ProcessThread
{
public:

    MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map, 
              const std::shared_ptr<std::mutex>& map_mutex,
              fastsense::CommandQueuePtr& q);

    void start(const Vector3i& pos, const fastsense::buffer::InputBuffer<PointHW>& points);

protected:
    void thread_run() override;

private:
    const std::shared_ptr<fastsense::map::LocalMap> local_map_;
    fastsense::kernels::TSDFKernel tsdf_krnl_;
    const std::shared_ptr<std::mutex> map_mutex_;
    std::mutex start_mutex_;
    bool running_;
    Vector3i pos_;
    std::unique_ptr<fastsense::buffer::InputBuffer<PointHW>> points_ptr_;
};

} // namespace fastsense::callback