/**
 * @file cloud_callback.cpp
 * @author Pascal Buschermöhle
 * @author Marc Eisoldt
 */

#include "cloud_callback.h"
#include <util/runtime_evaluator.h>
#include <msg/transform.h>
#include <algorithm>
#include <eigen3/Eigen/Geometry>
#include <tsdf/update_tsdf_hw.h>
#include <tsdf/krnl_tsdf_sw.h>
#include <util/logging/logger.h>

using namespace fastsense::callback;
using namespace fastsense::util;
using fastsense::util::logging::Logger;
using fastsense::buffer::InputBuffer;

CloudCallback::CloudCallback(Registration& registration,
                             const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                             const std::shared_ptr<LocalMap>& local_map,
                             const std::shared_ptr<GlobalMap>& global_map,
                             Matrix4f& pose,
                             const msg::TransformStampedBuffer::Ptr& transform_buffer,
                             const fastsense::CommandQueuePtr& q,
                             MapThread& map_thread,
                             std::mutex& map_mutex)
    : ProcessThread(),
      registration{registration},
      cloud_buffer{cloud_buffer},
      local_map{local_map},
      global_map{global_map},
      pose{pose},
      transform_buffer{transform_buffer},
      first_iteration{true},
      q{q},
      tsdf_krnl(q, local_map->getBuffer().size()),
      map_thread{map_thread},
      map_mutex{map_mutex}
{

}

void CloudCallback::thread_run()
{
    fastsense::msg::PointCloudPtrStamped point_cloud;
    auto& eval = RuntimeEvaluator::get_instance();
#ifdef TIME_MEASUREMENT
    int cnt = 0;
#endif
    while (running)
    {
        if (!cloud_buffer->pop_nb(&point_cloud, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        eval.start("total");
        eval.start("prep");

        //DO NOT TOUCH THE ORIGINAL POINT CLOUD, USE COPY INSTEAD
        fastsense::msg::PointCloudPtrStamped point_cloud2;
        point_cloud2.data_ = point_cloud.data_;
        point_cloud2.timestamp_ = point_cloud.timestamp_;

        preprocessor.median_filter(point_cloud2, 5);
        preprocessor.reduction_filter(point_cloud2);
        InputBuffer<PointHW> scan_point_buffer{q, point_cloud2.data_->points_.size()};
        preprocessor.preprocess_scan(point_cloud2, scan_point_buffer);

        eval.stop("prep");

        if (first_iteration)
        {
            first_iteration = false;

            int tau = ConfigManager::config().slam.max_distance();
            int max_weight = ConfigManager::config().slam.max_weight() * WEIGHT_RESOLUTION;
            tsdf_krnl.run(*local_map, scan_point_buffer, tau, max_weight);
            tsdf_krnl.waitComplete();
        }
        else
        {
            Matrix4f old_pose = pose;

            map_mutex.lock();
            eval.start("reg");
            registration.register_cloud(*local_map, scan_point_buffer, point_cloud2.timestamp_, pose);
            eval.stop("reg");
            map_mutex.unlock();

            // Logger::info("Pose:\n", std::fixed, std::setprecision(4), pose);

            if (std::isnan(pose(0, 0)))
            {
                Logger::error("Registration gave NaN");
                pose = old_pose;
            }
        }

        Vector3i pos((int)std::floor(pose(0, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(1, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(2, 3) / MAP_RESOLUTION));
        map_thread.go(pos, scan_point_buffer);

        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));

        // global_map->save_pose(pose(0, 3), pose(1, 3), pose(2, 3),
        //                       quat.x(), quat.y(), quat.z(), quat.w());

        msg::TransformStamped transform;
        transform.data_.translation = pose.block<3, 1>(0, 3);
        transform.data_.rotation = quat;
        transform_buffer->push_nb(transform, true);

        eval.stop("total");
#ifdef TIME_MEASUREMENT
        if (cnt == 20)
        {
            Logger::info(eval.to_string());
            cnt = 0;
        }
        cnt++;
#endif
    }
    Logger::info("Stopped Callback");
}
