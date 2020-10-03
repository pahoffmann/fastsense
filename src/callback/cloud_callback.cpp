/**
 * 
 * @author Pascal Buschermöhle
 * 
 */


#include "cloud_callback.h"

using namespace fastsense::callback;

CloudCallback::CloudCallback(Registration& registration, PointCloudBuffer& cloud_buffer)
     : ProcessThread(), 
     registration{registration}, 
     cloud_buffer{cloud_buffer}{}

void CloudCallback::start(){
    if(not running){
        running = true;
        worker = std::thread(&CloudCallback::callback, this);
    }
}

void CloudCallback::stop(){
    running = false;
}

void CloudCallback::callback(){
    while(running){
        fastsense::msg::PointCloudStamped point_cloud;
        cloud_buffer.pop(&point_cloud);

    }
}