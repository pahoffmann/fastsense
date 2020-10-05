/**
 * 
 * @author Pascal Buschermöhle
 * 
 */


#include "cloud_callback.h"

using namespace fastsense::callback;

CloudCallback::CloudCallback(Registration& registration, std::shared_ptr<PointCloudBuffer>& cloud_buffer, LocalMap& local_map, std::shared_ptr<GlobalMap>& global_map, Matrix4f& pose)
     : ProcessThread(), 
     registration{registration}, 
     cloud_buffer{cloud_buffer},
     local_map{local_map},
     global_map{global_map},
     pose{pose},
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

void CloudCallback::preprocess_scan(const fastsense::msg::PointCloudStamped& cloud, ScanPoints_t& scan_points, float map_resolution, const Matrix4f& pose){
    std::vector<fastsense::msg::Point> cloud_points = cloud.first->points_;
     
    Eigen::Vector4f v;

    for(unsigned int i = 0; i < cloud_points.size(); i++){
        Vector3f pointf;
        pointf << cloud_points[i].x, cloud_points[i].y, cloud_points[i].z;
        v << (pointf / map_resolution), 1.0f;
        pointf = (pose * v).block<3, 1>(0, 0);

        Vector3i point;
        point << std::floor(pointf.x()), std::floor(pointf.y()), std::floor(pointf.z());

        scan_points[i] = point;
    }
}

void CloudCallback::callback(){
    while(running){
        fastsense::msg::PointCloudStamped point_cloud;
        cloud_buffer->pop(&point_cloud);

        ScanPoints_t scan_points(point_cloud.first->points_.size());
        preprocess_scan(point_cloud, scan_points, ConfigManager::config().slam.map_resolution(), pose);

        if(first_iteration){
            first_iteration = false;
        }else{
            Matrix4f transform = registration.register_cloud(local_map, scan_points);
            pose = transform * pose;
            int x = (int)std::floor(pose(0, 3));
            int y = (int)std::floor(pose(1, 3));
            int z = (int)std::floor(pose(2, 3));
            local_map.shift(x, y, z);
        }

        Vector3i pose_integer;
        pose_integer << (int)std::floor(pose(0, 3)), (int)std::floor(pose(1, 3)), (int)std::floor(pose(2, 3));

        int tau = (int)(ConfigManager::config().slam.max_distance() / ConfigManager::config().slam.map_resolution());
        fastsense::tsdf::update_tsdf(scan_points, pose_integer, local_map, tau, ConfigManager::config().slam.max_weight());
    }
}