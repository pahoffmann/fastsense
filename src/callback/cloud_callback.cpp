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
                             fastsense::CommandQueuePtr& q)
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
      vis_buffer{vis_buffer}
{

}

Eigen::Vector3f Matrix4ToEuler(const Matrix4f& inputMat)
{
    Eigen::Matrix4d mat = inputMat.transpose().cast<double>();
    Eigen::Vector3d rPosTheta;

    double _trX, _trY;

    // Calculate Y-axis angle
    rPosTheta[1] = asin(std::max(-1.0, std::min(1.0, mat(2, 0)))); // asin returns nan for any number outside [-1, 1]
    if (mat(0, 0) <= 0.0)
    {
        rPosTheta[1] = M_PI - rPosTheta[1];
    }

    double C = cos(rPosTheta[1]);
    if (fabs( C ) > 0.005)                    // Gimbal lock?
    {
        _trX      =  mat(2, 2) / C;             // No, so get X-axis angle
        _trY      =  -mat(2, 1) / C;
        rPosTheta[0]  = atan2( _trY, _trX );
        _trX      =  mat(0, 0) / C;              // Get Z-axis angle
        _trY      = -mat(1, 0) / C;
        rPosTheta[2]  = atan2( _trY, _trX );
    }
    else                                        // Gimbal lock has occurred
    {
        rPosTheta[0] = 0.0;                       // Set X-axis angle to zero
        _trX      =  mat(1, 1);  //1                // And calculate Z-axis angle
        _trY      =  mat(0, 1);  //2
        rPosTheta[2]  = atan2( _trY, _trX );
    }
    return rPosTheta.cast<float>();
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

        bool tsdf_dirty = true;
        if (first_iteration)
        {
            first_iteration = false;
        }
        else
        {
            eval.start("reg");
            Matrix4f transform = registration.register_cloud(*local_map, scan_point_buffer);
            eval.stop("reg");

            Eigen::Quaternionf rotation(transform.block<3, 3>(0, 0));
            Vector3f pos = transform.block<3, 1>(0, 3);
            float angle = rotation.angularDistance(Eigen::Quaternionf::Identity());

            if (pos.norm() < MAP_RESOLUTION / 2 && angle < M_PI / 180)
            {
                tsdf_dirty = false;
            }
            else
            {
                pose = transform * pose;
                Logger::info("Pose:\n", std::fixed, std::setprecision(4), pose);

                int x = (int)std::floor(pose(0, 3) / MAP_RESOLUTION);
                int y = (int)std::floor(pose(1, 3) / MAP_RESOLUTION);
                int z = (int)std::floor(pose(2, 3) / MAP_RESOLUTION);

                eval.start("shift");
                local_map->shift(x, y, z);
                eval.stop("shift");
            }
        }

        int tau = (int)ConfigManager::config().slam.max_distance();

        if (tsdf_dirty)
        {
            eval.start("tsdf");
            tsdf_krnl.run(*local_map, scan_point_buffer, tau, ConfigManager::config().slam.max_weight());
            tsdf_krnl.waitComplete();

            // fastsense::tsdf::update_tsdf_hw(scan_point_buffer, local_map, tau, ConfigManager::config().slam.max_weight());

            // fastsense::buffer::InputOutputBuffer<std::pair<int, int>> new_entries(q, local_map->get_size().x() * local_map->get_size().y() * local_map->get_size().z());

            // for(int i = 0; i < local_map->get_size().x() * local_map->get_size().y() * local_map->get_size().z(); ++i)
            // {
            //     new_entries[i].first = 0;
            //     new_entries[i].second = 0;
            // }

            // std::vector<PointHW> kernel_points_sw(scan_point_buffer.size());

            // for(size_t i = 0; i < scan_point_buffer.size(); ++i)
            // {
            //     kernel_points_sw[i] = scan_point_buffer[i];
            // }

            // auto& size = local_map->get_size();
            // auto& pos = local_map->get_pos();
            // auto& offset = local_map->get_offset();


            // fastsense::tsdf::krnl_tsdf_sw(kernel_points_sw.data(),
            //                               kernel_points_sw.data(),
            //                               scan_point_buffer.size(),
            //                               local_map->getBuffer().getVirtualAddress(),
            //                               local_map->getBuffer().getVirtualAddress(),
            //                               size.x(), size.y(), size.z(),
            //                               pos.x(), pos.y(), pos.z(),
            //                               offset.x(), offset.y(), offset.z(),
            //                               new_entries.getVirtualAddress(),
            //                               new_entries.getVirtualAddress(),
            //                               tau,
            //                               ConfigManager::config().slam.max_weight());

            eval.stop("tsdf");
        }

        eval.start("vis");

        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        global_map->save_pose(pose(0, 3), pose(1, 3), pose(2, 3),
                              quat.x(), quat.y(), quat.z(), quat.w());

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
