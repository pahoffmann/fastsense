/**
 * @file imu_accumulator.cpp
 * @author Julian Gaal, Pascal Buschermöhle
 */

#include "imu_accumulator.h"
#include <util/time.h>

using namespace fastsense::registration;

ImuAccumulator::ImuAccumulator()
    :   first_imu_msg_{true},
        last_imu_timestamp_{}
{}

fastsense::Vector3f ImuAccumulator::rot_in_euler(const Matrix4f& m)
{
    return m.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
}

bool ImuAccumulator::imu_before_pcl(fastsense::util::HighResTimePoint& imu_ts, fastsense::util::HighResTimePoint& pcl_ts){
    return std::chrono::duration_cast<std::chrono::milliseconds>(pcl_ts - imu_ts).count() >= 0;
}

Eigen::Matrix4f ImuAccumulator::acc_transform(msg::ImuStampedBuffer& imu_buffer, util::HighResTimePoint pcl_timestamp) {
    
    msg::ImuStamped imu_msg;
    Matrix4f acc_transform = Matrix4f::Identity();
    
    auto keep_if = [&](msg::ImuStamped& msg){ return imu_before_pcl(msg.timestamp_, pcl_timestamp); };
    
    while(imu_buffer.pop_nb_if(&imu_msg, keep_if)) 
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
    const double acc_time = std::abs(std::chrono::duration_cast<util::time::secs_double>(imu_msg.timestamp_ - last_imu_timestamp_).count());
    Vector3f orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
    
    auto rotation = Eigen::AngleAxisf(orientation.x(), Vector3f::UnitX())
                    * Eigen::AngleAxisf(orientation.y(), Vector3f::UnitY())
                    * Eigen::AngleAxisf(orientation.z(), Vector3f::UnitZ());

    acc_transform.block<3, 3>(0, 0) = rotation.toRotationMatrix() * acc_transform.block<3, 3>(0, 0); //combine/update transforms
}
