#include <callback/map_thread.h>
#include <util/logging/logger.h>
#include <util/config/config_manager.h>

using fastsense::util::config::ConfigManager;
using fastsense::util::logging::Logger;

namespace fastsense::callback
{

MapThread::MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map, 
                     std::mutex& map_mutex,
                     fastsense::CommandQueuePtr& q)
    : ProcessThread(),
      local_map_(local_map),
      tsdf_krnl_(q, local_map->getBuffer().size()),
      map_mutex_(map_mutex),
      active_(false),
      reg_cnt_(0)
{
    /*
    Use the mutex as a 1-semaphore.
    wait = lock, signal = unlock.
    The mutex starts in the wrong state.
    */
    start_mutex_.lock();
}

void MapThread::go(const Vector3i& pos, const fastsense::buffer::InputBuffer<PointHW>& points)
{
    reg_cnt_++;
    const Vector3i& old_pos = local_map_->get_pos();
    float distance = ((pos.cast<float>() - old_pos.cast<float>()) * MAP_RESOLUTION).norm();

    // TODO: aus Config
    float param_pos = 0.5;
    int param_reg_cnt = 0;

    bool position_condition = distance > param_pos;
    bool reg_cnt_condition = param_reg_cnt > 0 && reg_cnt_ >= param_reg_cnt;
    if (!active_ && (position_condition || reg_cnt_condition))
    {
        pos_ = pos;
        points_ptr_.reset(new fastsense::buffer::InputBuffer<PointHW>(points));
        active_ = true;
        start_mutex_.unlock(); // signal
        reg_cnt_ = 0;
    }
}

void MapThread::thread_run()
{
    while (running)
    {
        start_mutex_.lock(); // wait
        Logger::info("Starting asynchronous map shift and tsdf update");

        local_map_->shift(pos_.x(), pos_.y(), pos_.z());
        
        tsdf_krnl_.run(*local_map_, *points_ptr_, (int) ConfigManager::config().slam.max_distance(), ConfigManager::config().slam.max_weight());
        tsdf_krnl_.waitComplete();

        Logger::info("Stopping asynchronous map shift and tsdf update");
        active_ = false;
    }
}

} // namespace fastsense::callback
