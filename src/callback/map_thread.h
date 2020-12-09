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

/**
 * @brief Encapsulates the async map shift and update
 */
class MapThread : public fastsense::util::ProcessThread
{
public:

    /**
     * @brief Construct a new Map Thread object
     * 
     * @param local_map Map, which should be updated in the thread
     * @param map_mutex  Synchronisation between the map thread and the cloud callback to prevent race conditions while using the map
     * @param tsdf_buffer Buffer for starting the visusalization thread
     * @param q Program command queue
     */
    MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map, 
              std::mutex& map_mutex,
              std::shared_ptr<TSDFBuffer> tsdf_buffer,
              fastsense::CommandQueuePtr& q);


    /**
     * @brief Don't allow to the make a copy of a map thread object
     */
    MapThread(const MapThread&) = delete;
    
    /**
     * @brief Don't allow to assign a map thread object     
     */
    MapThread& operator=(const MapThread&) = delete;

    /**
     * @brief Unlocks the thread for shifting und updating the map, 
     *        if a specific number of registartion periods were performed 
     *        or the position of the system has changed by a predefined threshold
     * 
     * @param pos Current position of the system
     * @param points  Current scanned points 
     */
    void go(const Vector3i& pos, const fastsense::buffer::InputBuffer<PointHW>& points);

    /**
     * @brief Stop the map thread safely
     */
    void stop() override;

protected:

    /**
     * @brief Shift and update the local map based on the current scanner data.
     *        Blocks as long as it is released by the go method
     */
    void thread_run() override;

private:

    /// Pointer to the current local map
    const std::shared_ptr<fastsense::map::LocalMap> local_map_;
    /// kernel object to perform an map update on hardware
    fastsense::kernels::TSDFKernel tsdf_krnl_;
    /// Synchronisation netween the cloud callcack and the map thread to prevent race conditions while using the local map
    std::mutex& map_mutex_;
    /// Mutex to control the when the map thread should be active
    std::mutex start_mutex_;
    /// Is the thread still active?
    std::atomic<bool> active_;
    /// System position at the activation time of the thread 
    Vector3i pos_;
    /// Scanner points ath the activation time of the thread
    std::unique_ptr<fastsense::buffer::InputBuffer<PointHW>> points_ptr_;
    /// Counter for the number of performed registartions after the last thread activation
    int reg_cnt_;
    /// Buffer for starting the map visualization thread
    std::shared_ptr<TSDFBuffer> tsdf_buffer_;
};

} // namespace fastsense::callback
