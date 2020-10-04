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
     pose{pose}{}

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

    }
}