/**
 * @file application.cpp
 * @author Marcel Flottmann
 */

#include <chrono>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>

#include "application.h"
#include <msg/imu.h>
#include <msg/tsdf_bridge_msg.h>
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <registration/registration.h>
#include <callback/cloud_callback.h>
#include <callback/imu_callback.h>
#include <map/local_map.h>
#include <map/global_map.h>
#include <comm/queue_bridge.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;
using namespace std::chrono_literals;

using fastsense::registration::Registration;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using fastsense::callback::CloudCallback;
using fastsense::callback::ImuCallback;

template<typename T>
class Runner
{
private:
    T& object;
public:
    Runner(T& obj) : object(obj)
    {
        object.start();
    }

    ~Runner()
    {
        object.stop();
    }
};

Application::Application()
    : signal_set{}
{
    // block signals
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigaddset(&signal_set, SIGTERM);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);

    Logger::info("Application initialized");
}

int Application::run()
{
    Logger::info("Application setup...");
    msg::ImuStampedBufferPtr imu_buffer = std::make_shared<msg::ImuStampedBuffer>(ConfigManager::config().imu.bufferSize());
    msg::ImuStampedBufferPtr imu_bridge_buffer = std::make_shared<msg::ImuStampedBuffer>(ConfigManager::config().imu.bufferSize());

    driver::Imu imu_driver{imu_buffer};
    comm::QueueBridge<msg::ImuStamped, true> imu_bridge(imu_buffer, imu_bridge_buffer, 5555);

    msg::PointCloudStampedBufferPtr pointcloud_buffer = std::make_shared<msg::PointCloudStampedBuffer>(ConfigManager::config().lidar.bufferSize());
    msg::PointCloudStampedBufferPtr pointcloud_bridge_buffer = std::make_shared<msg::PointCloudStampedBuffer>(ConfigManager::config().lidar.bufferSize());

    driver::VelodyneDriver lidar_driver{ConfigManager::config().lidar.port(), pointcloud_buffer};
    comm::QueueBridge<msg::PointCloudStamped, true> lidar_bridge(pointcloud_buffer, pointcloud_bridge_buffer, 7777);

    fastsense::CommandQueuePtr command_queue = fastsense::hw::FPGAManager::create_command_queue();
    Registration registration{command_queue, ConfigManager::config().registration.max_iterations(), ConfigManager::config().registration.it_weight_gradient()};
    std::shared_ptr<GlobalMap> global_map_ptr = std::make_shared<GlobalMap>("GlobalMap.h5", ConfigManager::config().slam.max_distance() / ConfigManager::config().slam.map_resolution(), ConfigManager::config().slam.initial_map_weight());
    LocalMap local_map{ConfigManager::config().slam.map_size_x(), ConfigManager::config().slam.map_size_y(), ConfigManager::config().slam.map_size_z(), global_map_ptr, command_queue};
    Matrix4f pose = Matrix4f::Identity();
    auto tsdf_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>>(2);
    auto transform_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::Transform>>(16);

    CloudCallback cloud_callback{registration, pointcloud_bridge_buffer, local_map, global_map_ptr, pose, tsdf_buffer, transform_buffer, command_queue};

    ImuCallback imu_callback{registration, imu_bridge_buffer};

    comm::QueueBridge<msg::TSDFBridgeMessage, true> tsdf_bridge{tsdf_buffer, nullptr, 6666};
    comm::QueueBridge<msg::Transform, true> transform_bridge{transform_buffer, nullptr, 8888};
    
    Logger::info("Application starting...");

    Runner run_lidar_driver(lidar_driver);
    Runner run_lidar_bridge(lidar_bridge);
    Runner run_imu_driver(imu_driver);
    Runner run_imu_bridge(imu_bridge);
    Runner run_cloud_callback{cloud_callback};
    Runner run_imu_callback{imu_callback};
    Runner run_tsdf_bridge(tsdf_bridge);
    Runner run_transform_bridge(transform_bridge);

    Logger::info("Application started!");

    int sig;
    int ret = sigwait(&signal_set, &sig);
    if (ret == -1)
    {
        auto err = errno;
        Logger::fatal("Wait for signal failed (", std::strerror(err), ")! Stopping Application...");
        return -1;
    }

    Logger::info("Stopping Application...");
    return 0;
}