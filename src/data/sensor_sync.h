#pragma once

/**
 * @author Julian Gaal
 */

#include <memory>

#include <util/params.h>
#include <util/process_thread.h>
#include <msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::data
{

using ImuStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::ImuMsgStamped>;
using ImuStampedBufferPtr = std::shared_ptr<fastsense::util::ConcurrentRingBuffer<fastsense::msg::ImuMsgStamped>>;
using PointCloudStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>;
using PointCloudStampedBufferPtr = std::shared_ptr<PointCloudStampedBuffer>;
using SyncedData = std::pair<fastsense::msg::ImuMsg, fastsense::msg::PointCloud::Ptr>;
using SyncedDataBuffer = fastsense::util::ConcurrentRingBuffer<SyncedData>;
using SyncedDataBufferPtr = std::shared_ptr<SyncedDataBuffer>;

class SensorSync : public fastsense::util::ProcessThread
{
public:
    SensorSync(ImuStampedBufferPtr imu_buffer, PointCloudStampedBufferPtr pointcloud_buffer, SyncedDataBufferPtr synced_data_buffer);

    void start() override;

    void sync();

    void stop() override;
private:
    ImuStampedBufferPtr imu_buffer_;
    PointCloudStampedBufferPtr pointcloud_buffer_;
    SyncedDataBufferPtr synced_data_buffer_;
    static constexpr int max_delay_ = fastsense::util::params::period_ / -2;
};

} // namespace fastsense::data