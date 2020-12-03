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
     * @brief Return acculated transform for given pointcloud timestamp
     *
     * @param buffer stamped imu buffer
     * @param pcl_timestamp timestamp of received pointcloud 
     * 
     * @return const Matrix4f the accumulated transform
     */
    Matrix4f acc_transform(msg::ImuStampedBuffer& buffer, util::HighResTimePoint pcl_timestamp);

    static fastsense::Vector3f rot_in_euler(const Matrix4f& m);

    /**
     * @brief Reset accumulated transform to identify matrix
     */
    void reset();

    //void update(const fastsense::msg::ImuStamped& imu);

    //void update(const fastsense::msg::Imu& imu, double acc_time);

private:
    void apply_transform(Matrix4f& acc_transform, const msg::ImuStamped& imu_msg);
    bool imu_before_pcl(util::HighResTimePoint& imu_ts, util::HighResTimePoint& pcl_ts);
    
    /// first imu message needs to be catched to calculate diff
    bool first_imu_msg_;

    /// last timestamp of imu to calculate difference to incoming messages
    util::HighResTimePoint last_imu_timestamp_;
};

} // namespace fastsense::registration