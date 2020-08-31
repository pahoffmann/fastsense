//
// Created by julian on 8/31/20.
//

#pragma once

#include <util/msg/point_cloud.h>
#include <util/msg/imu_msg.h>
#include <util/msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::data
{

using ImuStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::util::msg::ImuMsgStamped>;
using PointCloudStampedBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::util::msg::PointCloudStamped>;
using SyncedData = std::pair<fastsense::util::msg::ImuMsg, fastsense::util::msg::PointCloud>;

class SensorSync
{
public:
    SensorSync(ImuStampedBuffer& imu_buffer, PointCloudStampedBuffer& pointcloud_buffer);
    void pop(SyncedData& val);
private:
    ImuStampedBuffer& imu_buffer;
    PointCloudStampedBuffer& pointcloud_buffer;
};

} // namespace fastsense::data