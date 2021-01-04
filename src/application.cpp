/**
 * @file application.cpp
 * @author Marcel Flottmann
 */

#include <util/time.h>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>

#include "application.h"
#include <msg/imu.h>
#include <msg/tsdf_bridge_msg.h>
#include <msg/stamped.h>
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <util/runner.h>
#include <registration/registration.h>
#include <callback/cloud_callback.h>
#include <callback/map_thread.h>
#include <map/local_map.h>
#include <map/global_map.h>
#include <comm/queue_bridge.h>
#include <comm/buffered_receiver.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;
using namespace std::chrono_literals;

using fastsense::registration::Registration;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using fastsense::callback::CloudCallback;
using fastsense::callback::MapThread;

Application::Application()
    : signal_set{}
    , config{ConfigManager::config()}
{
    // block signals
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigaddset(&signal_set, SIGTERM);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);

    Logger::info("Application initialized");
}

std::unique_ptr<util::ProcessThread> Application::init_imu(msg::ImuStampedBuffer::Ptr imu_buffer)
{
    util::ProcessThread::UPtr imu_driver;
    if (config.bridge.use_from())
    {
        Logger::info("Launching BufferedImuReceiver");
        imu_driver.reset(new comm::BufferedImuStampedReceiver{config.bridge.host_from(), config.bridge.imu_port_from(), imu_buffer});
    }
    else
    {
        Logger::info("Launching Imu Driver");
        imu_driver.reset(new driver::Imu{imu_buffer});
    }

    return imu_driver;
}


std::unique_ptr<util::ProcessThread> Application::init_lidar(msg::PointCloudPtrStampedBuffer::Ptr pcl_buffer)
{
    util::ProcessThread::UPtr lidar_driver;

    if (config.bridge.use_from())
    {
        Logger::info("Launching BufferedPCLReceiver");
        lidar_driver.reset(new comm::BufferedPclStampedReceiver{config.bridge.host_from(), config.bridge.pcl_port_from(), pcl_buffer});
    }
    else
    {
        Logger::info("Launching Velodyne Driver");
        lidar_driver.reset(new driver::VelodyneDriver{config.lidar.port(), pcl_buffer});
    }

    return lidar_driver;
}

int Application::run()
{
    Logger::info("Application setup...");

    auto imu_buffer = std::make_shared<msg::ImuStampedBuffer>(config.imu.bufferSize());
    auto imu_bridge_buffer = std::make_shared<msg::ImuStampedBuffer>(config.imu.bufferSize());
    auto pointcloud_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(config.lidar.bufferSize());
    auto pointcloud_bridge_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(config.lidar.bufferSize());

    util::ProcessThread::UPtr imu_driver;
    util::ProcessThread::UPtr lidar_driver;

    if (config.bridge.use_from())
    {
        Logger::info("\n\n>>>>>>>>>> WARNING <<<<<<<<<<\n"
                     "Using Bridge as Input\n"
                     "Listening to Messages from ", config.bridge.host_from(), "\n"
                     "THIS HAS TO MATCH THE ADDRESS OF THE BRIDGE PC!\n"
                     ">>>>>>>>>> WARNING <<<<<<<<<<\n");
        imu_driver.reset(new comm::BufferedImuStampedReceiver{config.bridge.host_from(), config.bridge.imu_port_from(), imu_buffer});
        lidar_driver.reset(new comm::BufferedPclStampedReceiver{config.bridge.host_from(), config.bridge.pcl_port_from(), pointcloud_buffer});
    }
    else
    {
        Logger::info("Starting Sensors");
        imu_driver.reset(new driver::Imu{imu_buffer});
        lidar_driver.reset(new driver::VelodyneDriver{config.lidar.port(), pointcloud_buffer});
    }

    bool send = config.bridge.use_to();

    comm::QueueBridge<msg::ImuStamped, true> imu_bridge{imu_buffer, imu_bridge_buffer, config.bridge.imu_port_to(), send};
    comm::QueueBridge<msg::PointCloudPtrStamped, true> lidar_bridge{pointcloud_buffer, pointcloud_bridge_buffer, config.bridge.pcl_port_to(), send};

    auto command_queue = fastsense::hw::FPGAManager::create_command_queue();

    Registration registration{command_queue,
                              imu_bridge_buffer,
                              ConfigManager::config().registration.max_iterations(),
                              ConfigManager::config().registration.it_weight_gradient(),
                              ConfigManager::config().registration.epsilon()};

    auto global_map_ptr = std::make_shared<GlobalMap>(
                              "GlobalMap.h5",
                              config.slam.max_distance() / config.slam.map_resolution(),
                              config.slam.initial_map_weight());

    auto local_map = std::make_shared<LocalMap>(
                         config.slam.map_size_x(),
                         config.slam.map_size_y(),
                         config.slam.map_size_z(),
                         global_map_ptr, command_queue);

    Matrix4f pose = Matrix4f::Identity();
    auto tsdf_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>>(2);
    auto transform_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TransformStamped>>(16);
    auto vis_buffer = std::make_shared<util::ConcurrentRingBuffer<Matrix4f>>(2);

    std::mutex map_mutex;

    MapThread map_thread{local_map, map_mutex, tsdf_buffer, ConfigManager::config().slam.map_update_period(), ConfigManager::config().slam.map_update_position_threshold(), command_queue};

    CloudCallback cloud_callback{registration, pointcloud_bridge_buffer, local_map, global_map_ptr, pose, transform_buffer, command_queue, map_thread, map_mutex};

    comm::QueueBridge<msg::TSDFBridgeMessage, true> tsdf_bridge{tsdf_buffer, nullptr, 6666};
    comm::QueueBridge<msg::TransformStamped, true> transform_bridge{transform_buffer, nullptr, 8888};

    Logger::info("Application starting...");

    Runner run_lidar_driver(*lidar_driver);
    Runner run_lidar_bridge(lidar_bridge);
    Runner run_imu_driver(*imu_driver);
    Runner run_imu_bridge(imu_bridge);
    Runner run_cloud_callback{cloud_callback};
    Runner run_tsdf_bridge(tsdf_bridge);
    Runner run_transform_bridge(transform_bridge);
    Runner run_map_thread(map_thread);

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

    // ensure that last Scan has finished processing
    cloud_callback.stop();
    // save Map to Disk
    local_map->write_back();

    return 0;
}