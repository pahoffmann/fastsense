#pragma once

/**
 * @file imu_accumulator.h
 * @author Julian Gaal, Pascal Buschemoeller
 */

#include <util/time.h>
#include <mutex>
#include <util/point.h>
#include <msg/imu.h>

namespace fastsense::registration
{

/**
 * @brief Accumulates multiple ImuStamped messages to create an accumulated transform 
 * since the last PointCloud Message
 * 
 */
class ImuAccumulator
{
public:
    /**
     * @brief Construct a new Imu Accumulator object
     */
    ImuAccumulator();

    /**
     * @brief Destroy the Imu Accumulator object
     * 
     */
    ~ImuAccumulator() = default;

    /**
     * @brief Return acculated transform
     * 
     * @return const Matrix4f& the accumulated transform
     */
    const Matrix4f& acc_transform() const;

    /**
     * @brief Return accumulated transformation matrix in euler angles
     * 
     * @return fastsense::Vector3f vector of rotations in roll/pitch/yaw
     */
    fastsense::Vector3f rot_in_euler() const;

    /**
     * @brief Reset accumulated transform to identify matrix
     */
    void reset();

    //void update(const fastsense::msg::ImuStamped& imu);

    //void update(const fastsense::msg::Imu& imu, double acc_time);

    Matrix4f acc_transform(msg::ImuStampedBuffer& buffer, util::HighResTimePoint pcl_timestamp);

private:
    void apply_transform(const msg::ImuStamped& imu_msg);
    bool imu_before_pcl(util::HighResTimePoint& imu_ts, util::HighResTimePoint& pcl_ts);
    
    /// first imu message needs to be catched to calculate diff
    bool first_imu_msg_;

    /// accumulated transformation matrix
    Matrix4f acc_transform_;

    /// 
    Matrix4f local_transform_;

    /// last timestamp of imu to calculate difference to incoming messages
    util::HighResTimePoint last_imu_timestamp_;
};

} // namespace fastsense::registration

std::ostream& operator<<(std::ostream& os, const fastsense::registration::ImuAccumulator& acc);