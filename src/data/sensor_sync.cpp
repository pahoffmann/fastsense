//
// Created by julian on 8/31/20.
//

#include <data/sensor_sync.h>

using namespace fastsense::data;

SensorSync::SensorSync(ImuStampedBuffer& imu_buffer, PointCloudStampedBuffer& pointcloud_buffer) : imu_buffer(imu_buffer), pointcloud_buffer(pointcloud_buffer) {}

void pop(SyncedData& val)
{

}