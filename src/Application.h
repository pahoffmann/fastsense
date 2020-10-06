#pragma once

/**
 * @author Marcel Flottmann
 */

#include <driver/lidar/velodyne.h>
#include <driver/imu/imu.h>

namespace fastsense
{

class Application
{
private:
    sigset_t signal_set;

    data::ImuStampedBufferPtr imuBuffer;
    data::PointCloudStampedBufferPtr pointcloudBuffer;

    driver::Imu imuDriver;
    driver::VelodyneDriver lidarDriver;
public:
    Application();
    ~Application() = default;

    int run();
};

} // namespace fastsense