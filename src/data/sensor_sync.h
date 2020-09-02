//
// Created by julian on 8/31/20.
//

#pragma once

#include <memory>

#include <util/process_thread.h>
#include <util/msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::data
{

using ImuStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::util::msg::ImuMsgStamped>;
using ImuStampedBufferPtr = std::shared_ptr<fastsense::util::ConcurrentRingBuffer<fastsense::util::msg::ImuMsgStamped>>;
using PointCloudStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::util::msg::PointCloudStamped>;
using PointCloudStampedBufferPtr = std::shared_ptr<PointCloudStampedBuffer>;
using SyncedData = std::pair<fastsense::util::msg::ImuMsg::ptr, fastsense::util::msg::PointCloud::ptr>;
using SyncedDataBuffer = fastsense::util::ConcurrentRingBuffer<SyncedData>;
using SyncedDataBufferPtr = std::shared_ptr<SyncedDataBuffer>;

class SensorSync : public ProcessThread
{
public:
    SensorSync(ImuStampedBufferPtr imu_buffer, PointCloudStampedBufferPtr pointcloud_buffer, SyncedDataBufferPtr synced_data_buffer);
    /**
     *
     * @param val
     */
    void start() override;

    void sync();

    void stop() override;
private:
    ImuStampedBufferPtr imu_buffer_;
    PointCloudStampedBufferPtr pointcloud_buffer_;
    SyncedDataBufferPtr synced_data_buffer_;
};

} // namespace fastsense::data