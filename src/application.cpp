/**
 * @file application.cpp
 * @author Marcel Flottmann
 */

#include <util/time.h>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <gpiod.hpp>

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
#include <ui/button.h>
#include <ui/led.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;
using namespace std::chrono_literals;

using fastsense::callback::CloudCallback;
using fastsense::callback::MapThread;
using fastsense::map::GlobalMap;
using fastsense::map::LocalMap;
using fastsense::registration::Registration;

Application::Application()
    : config{ConfigManager::config()}
{
}

std::unique_ptr<util::ProcessThread> Application::init_imu(msg::ImuStampedBuffer::Ptr imu_buffer)
{
    std::chrono::milliseconds recv_timeout(config.bridge.recv_timeout());

    if (config.bridge.use_from())
    {
        Logger::info("Launching BufferedImuReceiver");
        Logger::info("\n\n>>>>>>>>>> WARNING <<<<<<<<<<\n"
                     "Using Bridge as Input\n"
                     "Listening to Messages from ",
                     config.bridge.host_from(), "\n"
                     "THIS HAS TO MATCH THE ADDRESS OF THE BRIDGE PC!\n"
                     ">>>>>>>>>> WARNING <<<<<<<<<<\n");
        return std::make_unique<comm::BufferedImuStampedReceiver>(
                   config.bridge.host_from(),
                   config.bridge.imu_port_from(),
                   recv_timeout,
                   imu_buffer);
    }
    else
    {
        Logger::info("Launching Imu Driver");
        return std::make_unique<driver::Imu>(imu_buffer, config.imu.filterSize());
    }
}

std::unique_ptr<util::ProcessThread> Application::init_lidar(msg::PointCloudPtrStampedBuffer::Ptr pcl_buffer)
{
    std::chrono::milliseconds recv_timeout(config.bridge.recv_timeout());

    if (config.bridge.use_from())
    {
        Logger::info("Launching BufferedPCLReceiver");
        return std::make_unique<comm::BufferedPclStampedReceiver>(
                   config.bridge.host_from(),
                   config.bridge.pcl_port_from(),
                   recv_timeout,
                   pcl_buffer);
    }
    else
    {
        Logger::info("Launching Velodyne Driver");
        return std::make_unique<driver::VelodyneDriver>(config.lidar.port(), pcl_buffer);
    }
}

int Application::run()
{
    Logger::info("Initialize Application...");
    // block signals
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigaddset(&signal_set, SIGTERM);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);

    auto imu_buffer = std::make_shared<msg::ImuStampedBuffer>(config.imu.bufferSize());
    auto imu_bridge_buffer = std::make_shared<msg::ImuStampedBuffer>(config.imu.bufferSize());
    auto pointcloud_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(config.lidar.bufferSize());
    auto pointcloud_bridge_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(config.lidar.bufferSize());

    util::ProcessThread::UPtr imu_driver = init_imu(imu_buffer);
    util::ProcessThread::UPtr lidar_driver = init_lidar(pointcloud_buffer);

    bool send = config.bridge.use_to();
    comm::QueueBridge<msg::ImuStamped, true> imu_bridge{imu_buffer, imu_bridge_buffer, config.bridge.imu_port_to(), send};
    comm::QueueBridge<msg::PointCloudPtrStamped, true> lidar_bridge{pointcloud_buffer, pointcloud_bridge_buffer, config.bridge.pcl_port_to(), send};

    auto command_queue = fastsense::hw::FPGAManager::create_command_queue();

    Registration registration{command_queue,
                              imu_bridge_buffer,
                              ConfigManager::config().registration.max_iterations(),
                              ConfigManager::config().registration.it_weight_gradient(),
                              ConfigManager::config().registration.epsilon()};

    Matrix4f pose = Matrix4f::Identity();
    auto tsdf_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TSDFBridgeMessage>>(2);
    auto transform_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TransformStamped>>(16);
    auto vis_buffer = std::make_shared<util::ConcurrentRingBuffer<Matrix4f>>(2);

    std::mutex map_mutex;

    MapThread map_thread{std::shared_ptr<LocalMap>(nullptr), map_mutex, tsdf_buffer, ConfigManager::config().slam.map_update_period(), ConfigManager::config().slam.map_update_position_threshold(), command_queue};

    CloudCallback cloud_callback{registration, pointcloud_bridge_buffer, std::shared_ptr<LocalMap>(nullptr), std::shared_ptr<GlobalMap>(nullptr), pose, transform_buffer, command_queue, map_thread, map_mutex};

    comm::QueueBridge<msg::TSDFBridgeMessage, true> tsdf_bridge{tsdf_buffer, nullptr, config.bridge.tsdf_port_to()};
    comm::QueueBridge<msg::TransformStamped, true> transform_bridge{transform_buffer, nullptr, config.bridge.transform_port_to()};

    gpiod::chip button_chip(config.gpio.button_chip());
    ui::Button button{button_chip.get_line(config.gpio.button_line())};

    gpiod::chip led_chip(config.gpio.led_chip());
    ui::Led led{led_chip.get_line(config.gpio.led_line())};

    Logger::info("Application initialized!");
    Logger::info("Wait for calibration...");

    auto running_condition = [&]()
    {
        timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 0;
        siginfo_t sig;
        int ret = sigtimedwait(&signal_set, &sig, &ts);
        if (ret >= 0)
        {
            Logger::info("Stopping Application...");
            return true;
        }
        else
        {
            auto err = errno;
            Logger::error("Wait for signal failed (", std::strerror(err), ")! Stopping Application...");
        }
        return false;
    };

    led.setFrequency(1.0);
    if (!button.wait_for_press_or_condition(running_condition))
    {
        return 0;
    }
    led.setFrequency(5.0);

    Logger::info("Calibrating IMU...");
    imu_driver->start();
    imu_driver->stop();
    Logger::info("IMU calibrated!");

    while (true)
    {
        Logger::info("Wait for SLAM start...");
        led.setOn();
        if (!button.wait_for_press_or_condition(running_condition))
        {
            return 0;
        }
        led.setFrequency(2.0);

        Logger::info("Starting SLAM...");

        std::ostringstream filename;
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        filename << "GlobalMap_" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S") << ".h5";
        auto global_map = std::make_shared<GlobalMap>(
                              filename.str(),
                              config.slam.max_distance() / config.slam.map_resolution(),
                              config.slam.initial_map_weight());

        auto local_map = std::make_shared<LocalMap>(
                             config.slam.map_size_x(),
                             config.slam.map_size_y(),
                             config.slam.map_size_z(),
                             global_map, command_queue);

        map_thread.set_local_map(local_map);
        cloud_callback.set_local_map(local_map);
        cloud_callback.set_global_map(global_map);
        {
            Runner run_lidar_driver(*lidar_driver);
            Runner run_lidar_bridge(lidar_bridge);
            Runner run_imu_driver(*imu_driver);
            Runner run_imu_bridge(imu_bridge);
            Runner run_cloud_callback{cloud_callback};
            Runner run_tsdf_bridge(tsdf_bridge);
            Runner run_transform_bridge(transform_bridge);
            Runner run_map_thread(map_thread);
            Logger::info("SLAM started! Running...");
            if (!button.wait_for_press_or_condition(running_condition))
            {
                return 0;
            }
        }
        Logger::info("SLAM stopped!");
        Logger::info("Write local and global map...");
        // save Map to Disk
        local_map->write_back();
        global_map->write_back();
        Logger::info("Local and global map saved!");
    }
}