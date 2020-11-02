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

using namespace fastsense::callback;
using namespace fastsense::util;
using fastsense::buffer::InputBuffer;

CloudCallback::CloudCallback(Registration& registration, const std::shared_ptr<PointCloudBuffer>& cloud_buffer, LocalMap& local_map, const std::shared_ptr<GlobalMap>& global_map, Matrix4f& pose,
                             const std::shared_ptr<TSDFBuffer>& tsdf_buffer, const std::shared_ptr<TransformBuffer>& transform_buffer, fastsense::CommandQueuePtr& q)
    : ProcessThread(),
      registration{registration},
      cloud_buffer{cloud_buffer},
      local_map{local_map},
      global_map{global_map},
      pose{pose},
      tsdf_buffer{tsdf_buffer},
      transform_buffer{transform_buffer},
      first_iteration{true},
      q{q},
      tsdf_krnl(q, local_map.getBuffer().size())
{

}

void CloudCallback::start()
{
    if (!running)
    {
        running = true;
        worker = std::thread(&CloudCallback::callback, this);
    }
}

void CloudCallback::stop()
{
    running = false;
}


void CloudCallback::callback()
{
    while (running)
    {
#ifdef TIME_MEASUREMENT
        auto& eval = RuntimeEvaluator::get_instance();
        eval.start("total");
        eval.start("init");
#endif

        fastsense::msg::PointCloudStamped point_cloud;
        cloud_buffer->pop(&point_cloud);
        fastsense::msg::PointCloudStamped point_cloud2;
        point_cloud2.first = std::make_shared<msg::PointCloud>(*point_cloud.first);
        point_cloud2.second = point_cloud.second;

        preprocessor.median_filter(point_cloud, 5);
        preprocessor.reduction_filter(point_cloud);
        InputBuffer<PointHW> scan_point_buffer{q, point_cloud.first->points_.size()};
        preprocessor.preprocess_scan(point_cloud, scan_point_buffer, pose);

#ifdef TIME_MEASUREMENT
        eval.stop("init");
#endif

        if (first_iteration)
        {
            first_iteration = false;
        }
        else
        {
#ifdef TIME_MEASUREMENT
            eval.start("reg");
#endif

            Matrix4f transform = registration.register_cloud(local_map, scan_point_buffer);

            std::cout << transform << std::endl;

#ifdef TIME_MEASUREMENT
            eval.stop("reg");
            eval.start("shift");
#endif

            pose = transform * pose;
            int x = (int)std::floor(pose(0, 3) / MAP_RESOLUTION);
            int y = (int)std::floor(pose(1, 3) / MAP_RESOLUTION);
            int z = (int)std::floor(pose(2, 3) / MAP_RESOLUTION);

            local_map.shift(x, y, z);

#ifdef TIME_MEASUREMENT
            eval.stop("shift");
#endif
        }

        int tau = (int)ConfigManager::config().slam.max_distance();

#ifdef TIME_MEASUREMENT
        eval.start("tsdf");
#endif

        tsdf_krnl.run(local_map, scan_point_buffer, tau, ConfigManager::config().slam.max_weight());
        tsdf_krnl.waitComplete();

        // fastsense::tsdf::update_tsdf_hw(scan_point_buffer, local_map, tau, ConfigManager::config().slam.max_weight());

        // fastsense::buffer::InputOutputBuffer<std::pair<int, int>> new_entries(q, local_map.get_size().x() * local_map.get_size().y() * local_map.get_size().z());

        // for(int i = 0; i < local_map.get_size().x() * local_map.get_size().y() * local_map.get_size().z(); ++i)
        // {
        //     new_entries[i].first = 0;
        //     new_entries[i].second = 0;
        // }

        // std::vector<PointHW> kernel_points_sw(scan_point_buffer.size());

        // for(size_t i = 0; i < scan_point_buffer.size(); ++i)
        // {
        //     kernel_points_sw[i] = scan_point_buffer[i];
        // }

        // fastsense::tsdf::krnl_tsdf_sw(kernel_points_sw.data(),
        //                             kernel_points_sw.data(),
        //                             scan_point_buffer.size(),
        //                             local_map.getBuffer(),
        //                             local_map.getBuffer(),
        //                             local_map.get_size().x(),
        //                             local_map.get_size().y(),
        //                             local_map.get_size().z(),
        //                             0,
        //                             0,
        //                             0,
        //                             local_map.get_offset().x(),
        //                             local_map.get_offset().y(),
        //                             local_map.get_offset().z(),
        //                             new_entries,
        //                             new_entries,
        //                             tau,
        //                             ConfigManager::config().slam.max_weight());

#ifdef TIME_MEASUREMENT
        eval.stop("tsdf");
        eval.start("vis");
#endif

        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        global_map->save_pose(pose(0, 3), pose(1, 3), pose(2, 3),
                              quat.x(), quat.y(), quat.z(), quat.w());

        msg::Transform transform;
        transform.translation = pose.block<3, 1>(0, 3);
        transform.rotation = quat;
        transform_buffer->push(transform);

        msg::TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = tau;
        tsdf_msg.size_ = local_map.get_size();
        tsdf_msg.pos_ = local_map.get_pos();
        tsdf_msg.offset_ = local_map.get_offset();
        tsdf_msg.tsdf_data_.reserve(local_map.getBuffer().size());
        std::copy(local_map.getBuffer().cbegin(), local_map.getBuffer().cend(), std::back_inserter(tsdf_msg.tsdf_data_));
        tsdf_buffer->push_nb(tsdf_msg, true);

#ifdef TIME_MEASUREMENT
        eval.stop("vis");
        eval.stop("total");
        std::cout << eval << std::endl;
#endif
    }
}