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
                             const msg::ImuStampedBuffer::Ptr& ground_truth_buffer,
                             bool send_after_registration,
                             const fastsense::CommandQueuePtr& q,
                             MapThread& map_thread,
                             std::mutex& map_mutex,
                             float point_scale)
    : ProcessThread(),
      registration{registration},
      cloud_buffer{cloud_buffer},
      local_map{local_map},
      global_map{global_map},
      pose{Matrix4f::Identity()},
      transform_buffer{transform_buffer},
      pointcloud_buffer{pointcloud_buffer},
      ground_truth_buffer(ground_truth_buffer),
      send_after_registration{send_after_registration},
      first_iteration{true},
      q{q},
      map_thread{map_thread},
      map_mutex{map_mutex},
      point_scale{point_scale},
      cloud_count(0),
      ground_count(0)
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
    fastsense::msg::ImuStamped ground_truth;

    std::unique_ptr<InputBuffer<PointHW>> scan_point_buffer;
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

        ++cloud_count;

        eval.start("total");
        
        point_cloud.data_->scaling_ = point_scale;
        int num_points = point_cloud.data_->points_.size();

        if (scan_point_buffer == nullptr || num_points > (int)scan_point_buffer->size())
        {
            // IMPORTANT: Delete old buffer first
            scan_point_buffer.reset();
            scan_point_buffer.reset(new InputBuffer<PointHW>(q, num_points * 1.5));
        }
        
        for (int i = 0; i < num_points; i++)
        {
            auto& point = point_cloud.data_->points_[i];
            scan_point_buffer->operator[](i) = PointHW(point.x(), point.y(), point.z());
        }

        if (ground_truth_buffer == nullptr)
        {
            if (first_iteration)
            {
                first_iteration = false;

                map_thread.get_tsdf_krnl().synchronized_run(*local_map, *scan_point_buffer, num_points);
            }
            else
            {
                Matrix4f old_pose = pose;

                map_mutex.lock();
                eval.start("reg");
                registration.register_cloud(*local_map, *scan_point_buffer, num_points, point_cloud.timestamp_, pose);
                eval.stop("reg");
                map_mutex.unlock();

                if (std::isnan(pose(0, 0)))
                {
                    Logger::error("Registration gave NaN");
                    pose = old_pose;
                }
            }
        }
        else
        {
            //std::cout << "Use ground truth!" << std::endl;

            if (ground_count < cloud_count && !ground_truth_buffer->pop_nb(&ground_truth, DEFAULT_POP_TIMEOUT))
            {
                eval.stop("total");
                continue;
            }

            ++ground_count;

            //std::cout << "Ground Truth: " << ground_truth.data_.ang.x() << " " << ground_truth.data_.ang.y() << " " << ground_truth.data_.ang.z() << " " 
            //                              << ground_truth.data_.acc.x() << " " << ground_truth.data_.acc.y() << " " << ground_truth.data_.acc.z() << std::endl;

            pose(0, 3) = ground_truth.data_.acc.x() * 1000;
            pose(1, 3) = ground_truth.data_.acc.y() * 1000;
            pose(2, 3) = ground_truth.data_.acc.z() * 1000;

            Eigen::Quaternionf q = Eigen::AngleAxisf(ground_truth.data_.ang.x(), Eigen::Vector3f::UnitX())
                                 * Eigen::AngleAxisf(ground_truth.data_.ang.y(), Eigen::Vector3f::UnitY())
                                 * Eigen::AngleAxisf(ground_truth.data_.ang.z(), Eigen::Vector3f::UnitZ());
            
            pose.block<3, 3>(0, 0) = q.toRotationMatrix();

            registration.transform_point_cloud(*scan_point_buffer, pose);
        }

        Vector3i pos((int)std::floor(pose(0, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(1, 3) / MAP_RESOLUTION),
                     (int)std::floor(pose(2, 3) / MAP_RESOLUTION));
        map_thread.go(pos, pose, *scan_point_buffer, num_points);

        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));

        msg::TransformStamped transform;
        transform.data_.translation = pose.block<3, 1>(0, 3);
        transform.data_.rotation = quat;
        transform.data_.scaling = point_scale;

        if (send_after_registration)
        {
            point_cloud.timestamp_ = util::HighResTime::now();
            pointcloud_buffer->push_nb(point_cloud, true);
        }

        transform.timestamp_ = point_cloud.timestamp_;
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
