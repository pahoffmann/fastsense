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
     * 
     * @param buffer stamped imu buffer
     */
    ImuAccumulator(msg::ImuStampedBuffer& buffer);

    /**
     * @brief Destroy the Imu Accumulator object
     * 
     */
    ~ImuAccumulator() = default;

    /**
     * @brief Return acculated transform for given pointcloud timestamp
     *
     * @param pcl_timestamp timestamp of received pointcloud 
     * 
     * @return const Matrix4f the accumulated transform
     */
    Matrix4f acc_transform(util::HighResTimePoint pcl_timestamp);

private:
    /**
     * @brief applies single imu transform with duration since last imu message to accumulation matrix
     * 
     * @param acc_transform current acculumation matrix
     * @param imu_msg stamped imu message
     */
    void apply_transform(Matrix4f& acc_transform, const msg::ImuStamped& imu_msg);
    bool before(util::HighResTimePoint& ts_1, util::HighResTimePoint& ts_2);
    /// imu buffer to use for accumulation
    msg::ImuStampedBuffer& buffer_;
    /// first imu message needs to be catched to calculate diff
    bool first_imu_msg_;

    /// last timestamp of imu to calculate difference to incoming messages
    util::HighResTimePoint last_imu_timestamp_;
};

} // namespace fastsense::registration