#pragma once

/**
 * @file application.h
 * @author Marcel Flottmann
 */

#include <driver/lidar/velodyne.h>
#include <driver/imu/imu.h>
#include <msg/imu.h>
#include <msg/point_cloud.h>
#include <util/config/config.h>
#include <util/process_thread.h>

namespace fastsense
{

class Application
{
private:
    util::config::Config& config;

    util::ProcessThread::UPtr init_imu(msg::ImuStampedBuffer::Ptr imu_buffer);
    util::ProcessThread::UPtr init_lidar(msg::PointCloudPtrStampedBuffer::Ptr pcl_buffer);

    void wait_for_signal();
    std::thread signal_thread;
    bool running;
public:
    Application();
    ~Application() = default;

    int run();
};

} // namespace fastsense