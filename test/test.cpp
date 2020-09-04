#include <driver/imu/imu.h>
#include <driver/lidar/velodyne.h>

#include <util/time_stamp.h>
#include <util/msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>
#include <util/logging/logger.h>
#include <util/config/config_manager.h>

#include <data/sensor_sync.h>

namespace fs = fastsense;
using namespace fs::driver;
using namespace fs::data;
using namespace fs::util::logging;
using namespace fs::util::config;

int main() {
    Logger::addSink(std::make_shared<sink::CoutSink>());
    Logger::setLoglevel(LogLevel::Debug);

    ConfigManager cm;
    cm.get().inner.addHandler([]{ Logger::info("inner updated"); });
    cm.load("test.json");

    std::cout << cm.get().asdf() << std::endl;

    ImuStampedBufferPtr imu_buffer = std::make_shared<ImuStampedBuffer>(1000);
    PointCloudStampedBufferPtr pointcloud_buffer = std::make_shared<PointCloudStampedBuffer>(1000);
    SyncedDataBufferPtr synced_data_buffer = std::make_shared<SyncedDataBuffer>(1000);

    Imu imu(imu_buffer);
    VelodyneDriver lidar("", 2368, pointcloud_buffer);
    fs::data::SensorSync synchronizer(imu_buffer, pointcloud_buffer, synced_data_buffer);

    lidar.start();
    imu.start();
    synchronizer.start();

    auto start = fs::util::TimeStamp();

    while (true) {
        fs::data::SyncedData data;
        synced_data_buffer->pop(&data);
        Logger::info("fps: ", 1000.f/(fs::util::TimeStamp() - start));
        start.reset();
     }

    return 0;
}
