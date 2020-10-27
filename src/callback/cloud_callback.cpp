/**
 *
 * @author Pascal Buschermöhle
 *
 */


#include "cloud_callback.h"
#include <util/runtime_evaluator.h>
#include <msg/transform.h>
#include <algorithm>
#include <eigen3/Eigen/Geometry>

using namespace fastsense::callback;
using namespace fastsense::util;
using fastsense::buffer::InputBuffer;
using fastsense::msg::Point;

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
      tsdf_krnl(q)
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

void CloudCallback::preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, InputBuffer<PointHW>& scan_points)
{
    const std::vector<fastsense::msg::Point>& cloud_points = cloud.first->points_;

    Eigen::Vector4f v;

    int scan_points_index = 0;
    for (unsigned int i = 0; i < cloud_points.size(); i++)
    {
        if (cloud_points[i].x == 0 && cloud_points[i].y == 0 && cloud_points[i].z == 0)
        {
            continue;
        }
        v << cloud_points[i].x, cloud_points[i].y, cloud_points[i].z, 1.0f;

        Vector3f pointf = (pose * v).block<3, 1>(0, 0);
        PointHW point{static_cast<int>(std::floor(pointf.x())),
                      static_cast<int>(std::floor(pointf.y())),
                      static_cast<int>(std::floor(pointf.z()))};

        scan_points[scan_points_index] = point;
        scan_points_index++;

    }
}

size_t CloudCallback::determineBufferSize(const fastsense::msg::PointCloudStamped& cloud)
{
    const std::vector<fastsense::msg::Point>& cloud_points = cloud.first->points_;

    return std::count_if(cloud_points.begin(), cloud_points.end(), [](const fastsense::msg::Point & p)
    {
        return p.x != 0 || p.y != 0 || p.z != 0;
    });
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

        InputBuffer<PointHW> scan_point_buffer{q, determineBufferSize(point_cloud)};
        preprocess_scan(point_cloud, scan_point_buffer);

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

        //fastsense::tsdf::update_tsdf_hw(scan_point_buffer, local_map, tau, ConfigManager::config().slam.max_weight());

#ifdef TIME_MEASUREMENT
        eval.stop("tsdf");
        eval.start("vis");
#endif

        Eigen::Quaternionf quat(pose.block<3, 3>(0,0));
        
        msg::Transform transform;
        transform.translation.x() = pose(0, 3);
        transform.translation.y() = pose(1, 3);
        transform.translation.z() = pose(2, 3);
        transform.rotation.x() = quat.x();
        transform.rotation.y() = quat.y();
        transform.rotation.z() = quat.z();
        transform.rotation.w = quat.w();
        transform_buffer->push(transform);

        msg::TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = tau;
        tsdf_msg.size_[0] = local_map.get_size().x();
        tsdf_msg.size_[1] = local_map.get_size().y();
        tsdf_msg.size_[2] = local_map.get_size().z();
        tsdf_msg.pos_[0] = local_map.get_pos().x();
        tsdf_msg.pos_[1] = local_map.get_pos().y();
        tsdf_msg.pos_[2] = local_map.get_pos().z();
        tsdf_msg.offset_[0] = local_map.get_offset().x();
        tsdf_msg.offset_[1] = local_map.get_offset().y();
        tsdf_msg.offset_[2] = local_map.get_offset().z();
        tsdf_msg.map_resolution_ = ConfigManager::config().slam.map_resolution();
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