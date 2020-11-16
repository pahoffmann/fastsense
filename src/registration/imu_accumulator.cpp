/**
 * @file imu_accululator.cpp
 * @author Julian Gaal
 */

#include "imu_accumulator.h"

using namespace fastsense::registration;

ImuAccumulator::ImuAccumulator()
    :   first_imu_msg_{true},
        combined_transform_{Matrix4f::Identity()},
        local_transform_{Matrix4f::Identity()},
        last_imu_timestamp_{}
{}

const Eigen::Matrix4f& ImuAccumulator::combined_transform() const
{
    return combined_transform_;
}

void ImuAccumulator::reset()
{
    combined_transform_.setIdentity();
}

void ImuAccumulator::update(const fastsense::msg::ImuStamped& imu)
{
    if (first_imu_msg_)
    {
        last_imu_timestamp_ = imu.second;
        first_imu_msg_ = false;
        return;
    }

    float acc_time = std::chrono::duration_cast<util::time::secs_float>(imu.second - last_imu_timestamp_).count();
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
    local_transform_.data();
    combined_transform_ = local_transform_ * combined_transform_; //combine/update transforms
}