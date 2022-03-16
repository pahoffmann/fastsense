#pragma once

/**
 * @file application.h
 * @author Marcel Flottmann
 */

#include <driver/lidar/velodyne.h>
#include <driver/imu/imu.h>
#include <msg/imu.h>
#include <msg/transform.h>
#include <msg/point_cloud.h>
#include <util/config/config.h>
#include <util/process_thread.h>

namespace fastsense
{

class Application
{
private:
    util::config::Config& config;

    std::unique_ptr<util::ProcessThread> init_imu(msg::ImuStampedBuffer::Ptr imu_buffer, bool use_phidgets);
    std::unique_ptr<util::ProcessThread>
    init_ouster(msg::PointCloudPtrStampedBuffer::Ptr pcl_buffer, msg::ImuStampedBuffer::Ptr imu_buffer,
                bool activate_imu);
    std::unique_ptr<util::ProcessThread> init_ground_truth(msg::ImuStampedBuffer::Ptr ground_truth_buffer);
public:
    Application();
    ~Application() = default;

    /// delete copy assignment operator
    Application& operator=(const Application& other) = delete;

    /// delete move assignment operator
    Application& operator=(Application&&) noexcept = delete;

    /// delete copy constructor
    Application(const Application&) = delete;

    /// delete move constructor
    Application(Application&&) = delete;

    int run();
};

} // namespace fastsense