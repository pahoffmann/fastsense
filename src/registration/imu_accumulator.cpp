/**
 * @file imu_accumulator.cpp
 * @author Julian Gaal, Pascal Buschermöhle
 */

#include "imu_accumulator.h"
#include <util/time.h>

using namespace fastsense::registration;
using namespace fastsense;

ImuAccumulator::ImuAccumulator(msg::ImuStampedBuffer::Ptr& buffer)
    :   buffer_{buffer},
        first_imu_msg_{true},
        last_imu_timestamp_{}
{}


bool ImuAccumulator::before(uint64_t ts_1, uint64_t ts_2){
    return ts_2 >= ts_1;
}

Eigen::Matrix4f ImuAccumulator::acc_transform(uint64_t pcl_timestamp) {
    
    msg::ImuStamped imu_msg;
    Matrix4f acc_transform = Matrix4f::Identity();
    
    auto imu_before_pcl = [&](msg::ImuStamped& msg){ return before(msg.timestamp_, pcl_timestamp); };

    while(buffer_->pop_nb_if(&imu_msg, imu_before_pcl)) 
    {
        if(first_imu_msg_)
        {
            last_imu_timestamp_ = imu_msg.timestamp_;
            first_imu_msg_ = false;
            continue;
        }
        
        apply_transform(acc_transform, imu_msg);
        last_imu_timestamp_ = imu_msg.timestamp_;
    }

    return acc_transform;
}

void ImuAccumulator::apply_transform(Matrix4f& acc_transform, const msg::ImuStamped& imu_msg)
{
    const auto& ang_vel = imu_msg.data_.ang;
    // relative timestamp is in milliseconds, convert into seconds
    const double acc_time = std::abs((imu_msg.timestamp_ - last_imu_timestamp_) / 1000.0);
    
    Vector3f orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
    
    auto rotation = Eigen::AngleAxisf(orientation.x(), Vector3f::UnitX())
                    * Eigen::AngleAxisf(orientation.y(), Vector3f::UnitY())
                    * Eigen::AngleAxisf(orientation.z(), Vector3f::UnitZ());

    Eigen::Matrix3f total = rotation.toRotationMatrix() * acc_transform.block<3, 3>(0, 0);
    acc_transform.block<3, 3>(0, 0) = total; //combine/update transforms
}
