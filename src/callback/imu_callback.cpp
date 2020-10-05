/**
 * 
 * @author Pascal Buschermöhle
 * 
 */


#include "imu_callback.h"

using namespace fastsense::callback;

ImuCallback::ImuCallback(Registration& registration, std::shared_ptr<ImuBuffer>& imu_buffer)
     : ProcessThread(), 
     registration{registration}, 
     imu_buffer{imu_buffer}{}

void ImuCallback::start(){
    if(not running){
        running = true;
        worker = std::thread(&ImuCallback::callback, this);
    }
}

void ImuCallback::stop(){
    running = false;
}

void ImuCallback::callback(){
    while(running){
        fastsense::msg::ImuMsgStamped imu;
        imu_buffer->pop(&imu);
        registration.update_imu_data(imu);
    }
}