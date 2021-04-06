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
#include <util/logging/logger.h>

using namespace fastsense::callback;
using namespace fastsense::util;
using fastsense::util::logging::Logger;
using fastsense::buffer::InputBuffer;

CloudCallback::CloudCallback(Registration& registration,
                             const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                             const std::shared_ptr<LocalMap>& local_map,
                             const std::shared_ptr<GlobalMap>& global_map,
                             const msg::TransformStampedBuffer::Ptr& transform_buffer,
                             const msg::PointCloudPtrStampedBuffer::Ptr& pointcloud_buffer,
                             bool send_after_registration,
                             const fastsense::CommandQueuePtr& q,
                             MapThread& map_thread,
                             std::mutex& map_mutex)
    : ProcessThread(),
      registration{registration},
      cloud_buffer{cloud_buffer},
      local_map{local_map},
      global_map{global_map},
      pose{Matrix4f::Identity()},
      transform_buffer{transform_buffer},
      pointcloud_buffer{pointcloud_buffer},
      send_after_registration{send_after_registration},
      first_iteration{true},
      q{q},
      map_thread{map_thread},
      map_mutex{map_mutex}
{

}

void CloudCallback::set_global_map(const std::shared_ptr<GlobalMap>& global_map)
{
    this->global_map = global_map;
}

void CloudCallback::set_local_map(const std::shared_ptr<LocalMap>& local_map)
{
    this->local_map = local_map;
}

void CloudCallback::thread_run()
{
    fastsense::msg::PointCloudPtrStamped point_cloud;
    std::unique_ptr<InputBuffer<PointHW>> scan_point_buffer;
    auto& eval = RuntimeEvaluator::get_instance();
#ifdef TIME_MEASUREMENT
    int cnt = 0;
#endif
    while (running)
    {
        // Wait until buffer is not empty
        if (!cloud_buffer->pop_nb(&point_cloud, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        eval.start("total");

        int num_points = point_cloud.data_->points_.size();

        // If InputBuffer does not yet exist or is not big enough, create one
        if (scan_point_buffer == nullptr || num_points > (int)scan_point_buffer->size())
        {
            // IMPORTANT: Delete old buffer first
            scan_point_buffer.reset();
            scan_point_buffer.reset(new InputBuffer<PointHW>(q, num_points * 1.5));
        }

        // Convert PointCloud points into Hardware Points and write them into the PointHW Buffer
        for (int i = 0; i < num_points; i++)
        {
            auto& point = point_cloud.data_->points_[i];
            scan_point_buffer->operator[](i) = PointHW(point.x(), point.y(), point.z());
        }

        if (first_iteration)
        {
            // For the first point cloud that is processed it is not possible to call registration because no map exists yet
            // So only the TSDF kernel is called to create an inital map to registrate against with following pointclouds
            first_iteration = false;

            map_thread.get_tsdf_krnl().synchronized_run(*local_map, *scan_point_buffer, num_points);
        }
        else
        {
            // This is called if its not the first pointcloud
            // In this case the registration is called
            Matrix4f old_pose = pose;

            map_mutex.lock();
            eval.start("reg");
            registration.register_cloud(*local_map, *scan_point_buffer, num_points, point_cloud.timestamp_, pose);
            eval.stop("reg");
            map_mutex.unlock();

            // Logger::info("Pose:\n", std::fixed, std::setprecision(4), pose);

            if (std::isnan(pose(0, 0)))
            {
                Logger::error("Registration gave NaN");
                pose = old_pose;
            }
        }

        // Call map thread with the current pose
        // map_thread.go() will shift the localmap and decide if a map update is necessary
        Vector3i pos((int)std::floor(pose(0, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(1, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(2, 3) / MAP_RESOLUTION));
        map_thread.go(pos, pose, *scan_point_buffer, num_points);

        // Write the current transform into the transform buffer which is used to send the transform to the host pc
        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        msg::TransformStamped transform;
        transform.data_.translation = pose.block<3, 1>(0, 3);
        transform.data_.rotation = quat;
        transform_buffer->push_nb(transform, true);

        if (send_after_registration)
        {
            point_cloud.timestamp_ = util::HighResTime::now();
            pointcloud_buffer->push_nb(point_cloud, true);
        }

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
