#include <driver/imu/imu.h>
#include <driver/lidar/velodyne.h>

#include <util/time_stamp.h>
#include <msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>
#include <util/logging/logger.h>

#include <data/sensor_sync.h>

#include "catch2_config.h"

namespace fs = fastsense;
using namespace fs::driver;
using namespace fs::data;
using namespace fs::util::logging;

TEST_CASE("sync sensor data", "[!hide][SensorSyncData")
{
    std::cout << "Testing 'sync sensor data'" << std::endl;
    Logger::addSink(std::make_shared<sink::CoutSink>());
    Logger::setLoglevel(LogLevel::Debug);

    ImuStampedBufferPtr imu_buffer = std::make_shared<ImuStampedBuffer>(1000);
    PointCloudStampedBufferPtr pointcloud_buffer = std::make_shared<PointCloudStampedBuffer>(1000);
    SyncedDataBufferPtr synced_data_buffer = std::make_shared<SyncedDataBuffer>(1000);

    Imu imu(imu_buffer);
    VelodyneDriver lidar(2368, pointcloud_buffer);
    fs::data::SensorSync synchronizer(imu_buffer, pointcloud_buffer, synced_data_buffer);

    lidar.start();
    imu.start();
    synchronizer.start();

    auto start = fs::util::TimeStamp();

    while (true)
    {
        fs::data::SyncedData data;
        synced_data_buffer->pop(&data);
        Logger::info("fps: ", 1000.f / (fs::util::TimeStamp() - start));
        start.reset();
    }
}
