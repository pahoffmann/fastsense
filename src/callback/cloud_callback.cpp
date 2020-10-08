/**
 * 
 * @author Pascal Buschermöhle
 * 
 */


#include "cloud_callback.h"
#include <algorithm>

using namespace fastsense::callback;

CloudCallback::CloudCallback(Registration& registration, const std::shared_ptr<PointCloudBuffer>& cloud_buffer, LocalMap& local_map, const std::shared_ptr<GlobalMap>& global_map, Matrix4f& pose, const std::shared_ptr<TSDFBuffer>& tsdf_buffer)
     : ProcessThread(), 
     registration{registration},
     cloud_buffer{cloud_buffer},
     local_map{local_map},
     global_map{global_map},
     pose{pose},
     tsdf_buffer{tsdf_buffer},
     first_iteration{true}{}

void CloudCallback::start(){
    if(not running){
        running = true;
        worker = std::thread(&CloudCallback::callback, this);
    }
}

void CloudCallback::stop(){
    running = false;
}

void CloudCallback::preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, ScanPoints_t& scan_points, const Matrix4f& pose){
    const std::vector<fastsense::msg::Point>& cloud_points = cloud.first->points_;
    
    Eigen::Vector4f v;

    for(unsigned int i = 0; i < cloud_points.size(); i++){
        if(cloud_points[i].x == 0 && cloud_points[i].y == 0 && cloud_points[i].z == 0){
            continue;
        }
        v << cloud_points[i].x, cloud_points[i].y, cloud_points[i].z, 1.0f;
        Vector3f pointf = (pose * v).block<3, 1>(0, 0);

        Vector3i point;
        point << std::floor(pointf.x()), std::floor(pointf.y()), std::floor(pointf.z());

        scan_points.push_back(point);
    }
}

void CloudCallback::callback(){
    while(running){
        fastsense::msg::PointCloudStamped point_cloud;
        cloud_buffer->pop(&point_cloud);

        ScanPoints_t scan_points;
        preprocess_scan(point_cloud, scan_points, pose);

        // if(first_iteration){
        //     first_iteration = false;
        // }else{
        //     Matrix4f transform = registration.register_cloud(local_map, scan_points);
        //     pose = transform * pose;
        //     int x = (int)std::floor(pose(0, 3));
        //     int y = (int)std::floor(pose(1, 3));
        //     int z = (int)std::floor(pose(2, 3));
        //     local_map.shift(x, y, z);
        // }

        Vector3i position_integer;
        position_integer << (int)std::floor(pose(0, 3)), (int)std::floor(pose(1, 3)), (int)std::floor(pose(2, 3));

        int tau = (int)ConfigManager::config().slam.max_distance();
        fastsense::tsdf::update_tsdf(scan_points, position_integer, local_map, tau, ConfigManager::config().slam.max_weight());

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
    }
}