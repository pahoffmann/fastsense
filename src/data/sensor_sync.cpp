//
// Created by julian on 8/31/20.
//

#include <util/msg/point_cloud.h>
#include <util/msg/imu_msg.h>
#include <util/time_stamp.h>
#include <data/sensor_sync.h>
#include <iostream>
#include "sensor_sync.h"


using namespace fastsense::data;
using fastsense::util::msg::PointCloudStamped;
using fastsense::util::msg::ImuMsgStamped;

SensorSync::SensorSync(ImuStampedBufferPtr imu_buffer, PointCloudStampedBufferPtr pointcloud_buffer, SyncedDataBufferPtr synced_data_buffer)
    :   ProcessThread(),
        imu_buffer_(imu_buffer),
        pointcloud_buffer_(pointcloud_buffer),
        synced_data_buffer_(synced_data_buffer) {}

void SensorSync::start()
{
    if (not running)
    {
        running = true;
        synced_data_buffer_->clear();
        worker = std::thread(&SensorSync::sync, this);
    }
}

void SensorSync::stop()
{
    running = false;
}

void SensorSync::sync()
{
    while (running)
    {
        PointCloudStamped pcs;
        ImuMsgStamped imus;

        if (pointcloud_buffer_->pop_nb(&pcs))
        {
            if (imu_buffer_->pop_nb(&imus))
            {
                auto& [pcl, time_pcl] = pcs;
                auto& [imu_msg, time_imu] = imus;

                std::cout << "diff: " << time_pcl - time_imu << "\n";
            }
        }
    }
}
