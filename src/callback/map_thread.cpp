/**
 * @file map_thread.cpp
 * @author Steffen Hinderink, Marc Eisoldt
 */

#include <callback/map_thread.h>
#include <util/logging/logger.h>
#include <util/config/config_manager.h>
#include <util/runtime_evaluator.h>

namespace fastsense::callback
{

using fastsense::util::config::ConfigManager;
using fastsense::util::logging::Logger;

MapThread::MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map,
                     std::mutex& map_mutex,
                     std::shared_ptr<TSDFBuffer> tsdf_buffer,
                     unsigned int period,
                     float position_threshold,
                     fastsense::CommandQueuePtr& q)
    : ProcessThread(),
      local_map_(local_map),
      tsdf_krnl_(q, local_map->getBuffer().size()),
      map_mutex_(map_mutex),
      active_(false),
      period_(period),
      position_threshold_(position_threshold),
      reg_cnt_(0),
      tsdf_buffer_(tsdf_buffer)
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

    bool position_condition = distance > position_threshold_;
    bool reg_cnt_condition = period_ > 0 && reg_cnt_ >= period_;
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
    util::RuntimeEvaluator eval;

    while (running)
    {
        start_mutex_.lock();
        if (!running)
        {
            break;
        }
        Logger::info("Starting SUV");

        eval.start("copy");
        map::LocalMap tmp_map(*local_map_);
        eval.stop("copy");

        // shift
        eval.start("shift");
        tmp_map.shift(pos_.x(), pos_.y(), pos_.z());
        eval.stop("shift");

        // tsdf update
        eval.start("tsdf");
        int tau = ConfigManager::config().slam.max_distance();
        int max_weight = ConfigManager::config().slam.max_weight() * WEIGHT_RESOLUTION;
        tsdf_krnl_.run(tmp_map, *points_ptr_, tau, max_weight);
        tsdf_krnl_.waitComplete();
        eval.stop("tsdf");

        map_mutex_.lock();
        *local_map_ = std::move(tmp_map);
        map_mutex_.unlock();

        // visualize
        eval.start("vis");
        msg::TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = tau;
        tsdf_msg.size_ = local_map_->get_size();
        tsdf_msg.pos_ = local_map_->get_pos();
        tsdf_msg.offset_ = local_map_->get_offset();
        tsdf_msg.tsdf_data_.reserve(local_map_->getBuffer().size());
        std::copy(local_map_->getBuffer().cbegin(), local_map_->getBuffer().cend(), std::back_inserter(tsdf_msg.tsdf_data_));
        tsdf_buffer_->push_nb(tsdf_msg, true);
        eval.stop("vis");

        Logger::info("Map Thread:\n", eval.to_string(), "\nStopping SUV");
        active_ = false;
    }
}

void MapThread::stop()
{
    if (running && worker.joinable())
    {
        running = false;
        start_mutex_.unlock();
        worker.join();
    }
}

} // namespace fastsense::callback
