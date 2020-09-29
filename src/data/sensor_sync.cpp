//
// Created by julian on 8/31/20.
//

#include <msg/point_cloud.h>
#include <msg/imu_msg.h>
#include <util/time_stamp.h>
#include <data/sensor_sync.h>
#include <iostream>
#include <data/sensor_sync.h>


using namespace fastsense::data;
using fastsense::msg::PointCloudStamped;
using fastsense::msg::PointCloud;
using fastsense::msg::ImuMsgStamped;
using fastsense::msg::ImuMsg;
using fastsense::util::TimeStamp;

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

        TimeStamp imu_timestamp, pcl_timestamp;
        PointCloud::Ptr pcl;
        ImuMsg imu_msg;

        // "referenzdaten" kommen aus laserscanner, da er viel langsamer ist
        pointcloud_buffer_->pop(&pcs);
        std::tie(pcl, pcl_timestamp) = pcs;

        // Idee: imu_buffer zeit - pointcloud_buffer zeit -> wie viel vorher die letzte Imu messung war
        do
        {
            imu_buffer_->pop(&imus);
            std::tie(imu_msg, imu_timestamp) = imus;
        }
        while (imu_timestamp - pcl_timestamp < max_delay_);

        synced_data_buffer_->push_nb(std::make_pair(imu_msg, pcl), true);
    }
}
