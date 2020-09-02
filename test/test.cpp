#include <driver/imu/imu.h>
#include <driver/lidar/velodyne.h>

#include <util/time_stamp.h>
#include <util/msg/msgs_stamped.h>
#include <util/concurrent_ring_buffer.h>

#include <data/sensor_sync.h>

namespace fs = fastsense;
using namespace fs::driver;
using namespace fs::data;

int main() {
    ImuStampedBufferPtr imu_buffer = std::make_shared<ImuStampedBuffer>(1000);
    PointCloudStampedBufferPtr pointcloud_buffer = std::make_shared<PointCloudStampedBuffer>(1000);
    SyncedDataBufferPtr synced_data_buffer = std::make_shared<SyncedDataBuffer>(1000);

    Imu imu(imu_buffer);
    VelodyneDriver lidar("", 2368, pointcloud_buffer);
    fs::data::SensorSync synchronizer(imu_buffer, pointcloud_buffer, synced_data_buffer);

    imu.start();
    lidar.start();
    synchronizer.start();

    while (true) {
//        fs::data::SyncedData data;
//        auto test = synchronizer.pop_nb(data);

        std::cout << "--\n"
                  << "pcl buffer: " << pointcloud_buffer->getLength() << "\n"
                  << "imu buffer: " << imu_buffer->getLength() << "\n"
                  << "synced_data_buffer: " << synced_data_buffer->getLength() << "\n";
//                  << "synced: " << (test ? "true" : "false") << "\n";
     };
    return 0;
}
