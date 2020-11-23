/**
 * @file imu_accululator.cpp
 * @author Julian Gaal
 */

#include "imu_accumulator.h"
#include <iomanip>
#include <iostream>

using namespace fastsense::registration;
// TODO first imu msg immer ture, auch wenn von ROS 
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

void ImuAccumulator::update(const fastsense::msg::ImuStamped& imu)
{
    if (first_imu_msg_)
    {
        last_imu_timestamp_ = imu.second;
        first_imu_msg_ = false;
        return;
    }

    double acc_time = std::chrono::duration_cast<util::time::secs_double>(imu.second - last_imu_timestamp_).count();
    apply_transform(acc_time, imu.first.ang);
    last_imu_timestamp_ = imu.second;
}

void ImuAccumulator::update(const fastsense::msg::Imu& imu, double acc_time)
{
    apply_transform(acc_time, imu.ang);
}

void ImuAccumulator::apply_transform(double acc_time, const Vector3f& ang_vel)
{
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
