#pragma once

/**
 * @file imu_accululator.h
 * @author Julian Gaal
 */

#include <mutex>
#include <util/point.h>
#include <util/time_stamp.h>
#include <msg/imu.h>

namespace fastsense::registration
{

class ImuAccumulator
{
public:
    ImuAccumulator()
        :   mutex_{},
            first_imu_msg_{true},
            imu_accumulated_{},
            local_transform_{Matrix4f::Identity()},
            curr_imu_timestamp_{}
    {}

    ~ImuAccumulator() = default;

    const Eigen::Matrix4f& acc()
    {
        return imu_accumulated_;
    }

    void reset()
    {
        imu_accumulated_.setZero();
    }

    void update(const fastsense::msg::ImuStamped& imu)
    {
        if (first_imu_msg_)
        {
            curr_imu_timestamp_ = imu.second;
            first_imu_msg_ = false;
            return;
        }

        float acc_time = std::chrono::duration_cast<std::chrono::milliseconds>(imu.second - curr_imu_timestamp_).count() / 1000.0f;

        curr_imu_timestamp_ = imu.second;
    }

private:
    void do_update(float acc_time, const Vector3f ang_vel)
    {
        Vector3f orientation = ang_vel * acc_time; //in radiants [rad, rad, rad]
        auto rotation =   Eigen::AngleAxisf(orientation.x(), Vector3f::UnitX())
                          * Eigen::AngleAxisf(orientation.y(), Vector3f::UnitY())
                          * Eigen::AngleAxisf(orientation.z(), Vector3f::UnitZ());

        local_transform_.block<3, 3>(0, 0) = rotation.toRotationMatrix();

        mutex_.lock();
        imu_accumulated_ = local_transform_ * imu_accumulated_; //combine/update transforms
        mutex_.unlock();
    }

    std::mutex mutex_;
    bool first_imu_msg_;
    Eigen::Matrix4f imu_accumulated_;
    Matrix4f local_transform_;
    util::TimeStamp curr_imu_timestamp_;
};

} // namespace fastsense::registration