/**
 * @file imu_accumulator.cpp
 * @author Julian Gaal, Pascal Buschermoeller
 */

#include "imu_accumulator.h"
#include "util/time.h"
#include <iomanip>
#include <iostream>

using namespace fastsense::registration;

ImuAccumulator::ImuAccumulator()
    :   first_imu_msg_{true},
        acc_transform_{Matrix4f::Identity()},
        local_transform_{Matrix4f::Identity()},
        last_imu_timestamp_{}
{}

const Eigen::Matrix4f& ImuAccumulator::acc_transform() const
{
    return acc_transform_;
}

fastsense::Vector3f ImuAccumulator::rot_in_euler() const
{
    return acc_transform_.block<3, 3>(0, 0).eulerAngles(0, 1, 2) ;
}

void ImuAccumulator::reset()
{
    acc_transform_.setIdentity();
}

/*void ImuAccumulator::update(const fastsense::msg::ImuStamped& imu)
{
    if (first_imu_msg_)
    {
        last_imu_timestamp_ = imu.timestamp_;
        first_imu_msg_ = false;
        return;
    }

    double acc_time = std::chrono::duration_cast<util::time::secs_double>(imu.timestamp_ - last_imu_timestamp_).count();
    apply_transform(acc_time, imu.data_.ang);
    last_imu_timestamp_ = imu.timestamp_;
}

void ImuAccumulator::update(const fastsense::msg::Imu& imu, double acc_time)
{
    apply_transform(acc_time, imu.ang);
}
*/

bool ImuAccumulator::imu_before_pcl(fastsense::util::HighResTimePoint& imu_ts, fastsense::util::HighResTimePoint& pcl_ts){
    return std::chrono::duration_cast<std::chrono::milliseconds>(pcl_ts - imu_ts).count() >= 0;
}

Eigen::Matrix4f ImuAccumulator::acc_transform(msg::ImuStampedBuffer& imu_buffer, util::HighResTimePoint pcl_timestamp){
    msg::ImuStamped imu_msg;
    std::cout << "got here" << std::endl;
    while(imu_buffer.pop_nb(&imu_msg) && imu_before_pcl(imu_msg.timestamp_, pcl_timestamp)){
        std::cout << "got here too" << std::endl;
        if(first_imu_msg_){
            last_imu_timestamp_ = imu_msg.timestamp_;
            first_imu_msg_ = false;
            continue;
        }
        apply_transform(imu_msg);
        last_imu_timestamp_ = imu_msg.timestamp_;
    }

    Matrix4f acc_result = acc_transform_;
    reset();
    apply_transform(imu_msg);

    return acc_result;
}

void ImuAccumulator::apply_transform(const msg::ImuStamped& imu_msg)
{
    const auto& ang_vel = imu_msg.data_.ang;
    const double acc_time = std::chrono::duration_cast<util::time::secs_double>(imu_msg.timestamp_ - last_imu_timestamp_).count();
    Vector3f orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
    auto rotation =   Eigen::AngleAxisf(orientation.x(), Vector3f::UnitX())
                        * Eigen::AngleAxisf(orientation.y(), Vector3f::UnitY())
                        * Eigen::AngleAxisf(orientation.z(), Vector3f::UnitZ());

    local_transform_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    acc_transform_ = local_transform_ * acc_transform_; //combine/update transforms
}

std::ostream& operator<<(std::ostream& os, const ImuAccumulator& acc)
{
    const auto& rotation = acc.rot_in_euler();
    os.precision(4);
    os.width(5);
    os << std::fixed;
    os << "\n" << rotation.x() << "\n" << rotation.y() << "\n" << rotation.z() << "\n";
    return os;
}
