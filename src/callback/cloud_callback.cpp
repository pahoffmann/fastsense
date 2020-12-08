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
                             const VisPublisher::BufferPtr& vis_buffer,
                             const std::shared_ptr<TransformBuffer>& transform_buffer,
                             fastsense::CommandQueuePtr& q,
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
      vis_buffer{vis_buffer},
      map_thread{map_thread},
      map_mutex{map_mutex}
{

}

void CloudCallback::thread_run()
{
    fastsense::msg::PointCloudStamped point_cloud;
    auto& eval = RuntimeEvaluator::get_instance();
    while (running)
    {
        if (!cloud_buffer->pop_nb(&point_cloud, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        eval.start("total");
        eval.start("prep");

        fastsense::msg::PointCloudStamped point_cloud2;
        point_cloud2.first = std::make_shared<msg::PointCloud>(*point_cloud.first);
        point_cloud2.second = point_cloud.second;

        preprocessor.median_filter(point_cloud2, 5);
        preprocessor.reduction_filter(point_cloud2);
        InputBuffer<PointHW> scan_point_buffer{q, point_cloud2.first->points_.size()};
        preprocessor.preprocess_scan(point_cloud2, scan_point_buffer, pose);

        eval.stop("prep");

        if (first_iteration)
        {
            first_iteration = false;
        }
        else
        {
            eval.start("reg");
            Matrix4f transform = registration.register_cloud(*local_map, scan_point_buffer);
            eval.stop("reg");

            pose = transform * pose;
            Logger::info("Pose:\n", std::fixed, std::setprecision(4), pose);
        }

        map_mutex.lock();
        Vector3i pos((int)std::floor(pose(0, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(1, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(2, 3) / MAP_RESOLUTION));
        map_thread.go(pos, scan_point_buffer);
        map_mutex.unlock();

        eval.start("vis");

        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        
        // global_map->save_pose(pose(0, 3), pose(1, 3), pose(2, 3),
        //                       quat.x(), quat.y(), quat.z(), quat.w());

        msg::Transform transform;
        transform.translation = pose.block<3, 1>(0, 3);
        transform.rotation = quat;
        transform_buffer->push_nb(transform, true);

        vis_buffer->push_nb(pose, true);

        eval.stop("vis");
        eval.stop("total");
#ifdef TIME_MEASUREMENT
        Logger::info(eval.to_string());
#endif
    }
    Logger::info("Stopped Callback");
}

void VisPublisher::thread_run()
{
    Matrix4f val;
    while (running)
    {
        if (!vis_buffer->pop_nb(&val, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        msg::TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = ConfigManager::config().slam.max_distance();
        tsdf_msg.size_ = local_map->get_size();
        tsdf_msg.pos_ = local_map->get_pos();
        tsdf_msg.offset_ = local_map->get_offset();
        tsdf_msg.tsdf_data_.reserve(local_map->getBuffer().size());
        std::copy(local_map->getBuffer().cbegin(), local_map->getBuffer().cend(), std::back_inserter(tsdf_msg.tsdf_data_));
        tsdf_buffer->push_nb(tsdf_msg, true);
    }
}
