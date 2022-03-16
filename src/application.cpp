/**
 * @file application.cpp
 * @author Marcel Flottmann
 */

#include <util/time.h>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <gpiod.hpp>

#include "application.h"
#include <msg/imu.h>
#include <msg/stamped.h>
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <util/runner.h>
#include <registration/registration.h>
#include <preprocessing/preprocessing.h>
#include <callback/cloud_callback.h>
#include <callback/map_thread.h>
#include <map/local_map.h>
#include <map/global_map.h>
#include <comm/queue_bridge.h>
#include <comm/buffered_receiver.h>
#include <ui/button.h>
#include <ui/led.h>

#include <driver/lidar/ouster.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;
using namespace std::chrono_literals;

using fastsense::callback::CloudCallback;
using fastsense::callback::MapThread;
using fastsense::map::GlobalMap;
using fastsense::map::LocalMap;
using fastsense::registration::Registration;
using fastsense::preprocessing::Preprocessing;

Application::Application()
    : config{ConfigManager::config()}
{
}

std::unique_ptr<util::ProcessThread> Application::init_imu(msg::ImuStampedBuffer::Ptr imu_buffer, bool use_phidgets)
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
        if (!use_phidgets)
        {
            Logger::info("Skipping Phidgets Driver");
            return nullptr;
        }

        const size_t filter_size = config.imu.filterSize();
        if (filter_size < 2)
        {
            throw std::runtime_error("Imu window filter < 2 make no sense, du muruk.");
        }

        Logger::info("Launching Phidgets Driver");
        return std::make_unique<driver::Imu>(imu_buffer, config.imu.filterSize());
    }
}

std::unique_ptr<util::ProcessThread>
Application::init_ouster(msg::PointCloudPtrStampedBuffer::Ptr pcl_buffer, msg::ImuStampedBuffer::Ptr imu_buffer,
                         bool activate_imu)
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
    else {
        double height = std::log2(config.lidar.height());
        int height_int = static_cast<int>(height);
        if (!(height > 0 && std::abs(height - height_int) == 0 && height_int >= 4 && height_int <= 7)) {
            throw std::runtime_error(
                    "Only 16, 32, 64 or 128 lidar rings are supported options to reduce horizontal lines");
        }

        const std::string &lidar_mode_arg = config.lidar.mode();
        auto lidar_mode = ouster::sensor::lidar_mode_of_string(lidar_mode_arg);
        if (!lidar_mode)
        {
            throw std::runtime_error("Invalid lidar mode! 512x10, 512x20, 1024x10, 1024x20, or 2048x10 are allowed");
        }

        const size_t filter_size = config.imu.filterSize();
        if (filter_size < 2)
        {
            throw std::runtime_error("Imu window filter < 2 make no sense, du muruk.");
        }

        if (activate_imu) {
            Logger::info("Launching Ouster Driver including IMU");
            return std::make_unique<driver::OusterDriver>("192.168.1.235", pcl_buffer, lidar_mode, imu_buffer,
                                                          filter_size);
        }

        Logger::info("Launching Ouster Driver *ex*cluding IMU");
        return std::make_unique<driver::OusterDriver>("192.168.1.235", pcl_buffer, lidar_mode);
    }
}

std::unique_ptr<util::ProcessThread> Application::init_ground_truth(msg::ImuStampedBuffer::Ptr ground_truth_buffer)
{
    std::chrono::milliseconds recv_timeout(config.bridge.recv_timeout());

    if (config.bridge.use_from())
    {
        return std::make_unique<comm::BufferedImuStampedReceiver>(
                   config.bridge.host_from(),
                   4440,
                   recv_timeout,
                   ground_truth_buffer);    
    }

    return nullptr;
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
    auto pointcloud_send_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(1);

    auto ground_truth_buffer = std::make_shared<msg::ImuStampedBuffer>(config.lidar.bufferSize());
    auto further_ground_truth_buffer = std::make_shared<msg::ImuStampedBuffer>(config.lidar.bufferSize());

    bool use_phidgets = config.imu.use_phidgets();
    util::ProcessThread::UPtr imu_driver = init_imu(imu_buffer, use_phidgets);
    util::ProcessThread::UPtr lidar_driver = init_ouster(pointcloud_buffer, imu_buffer, !use_phidgets);
    util::ProcessThread::UPtr ground_truth_driver = init_ground_truth(ground_truth_buffer);

    bool send = config.bridge.use_to();
    comm::QueueBridge<msg::ImuStamped, true> imu_bridge{imu_buffer, imu_bridge_buffer, config.bridge.imu_port_to(), send};

    bool send_original = config.bridge.send_original();
    bool send_preprocessed = config.bridge.send_preprocessed();
    bool send_after_registration = config.bridge.send_after_registration();
    if (send_original + send_preprocessed + send_after_registration > 1)
    {
        throw std::runtime_error("More than one send option active in config.json/bridge/send_*");
    }

    const float& point_scale = config.lidar.pointScale();

    Preprocessing preprocessing{pointcloud_buffer,
                                pointcloud_bridge_buffer,
                                pointcloud_send_buffer,
                                ground_truth_buffer,
                                further_ground_truth_buffer,
                                send_original,
                                send_preprocessed,
                                point_scale};

    auto command_queue = fastsense::hw::FPGAManager::create_command_queue();

    Registration registration{command_queue,
                              imu_bridge_buffer,
                              config.registration.max_iterations(),
                              config.registration.it_weight_gradient(),
                              config.registration.epsilon()};

    int tau = config.slam.max_distance();
    int max_weight = config.slam.max_weight() * WEIGHT_RESOLUTION;
    int initial_weight = config.slam.initial_map_weight() * WEIGHT_RESOLUTION;
    assert(tau >= std::numeric_limits<TSDFEntry::ValueType>::min());
    assert(tau <= std::numeric_limits<TSDFEntry::ValueType>::max());
    assert(max_weight >= 0);
    assert(max_weight <= std::numeric_limits<TSDFEntry::WeightType>::max());
    assert(initial_weight >= 0);
    assert(initial_weight <= std::numeric_limits<TSDFEntry::WeightType>::max());

    auto transform_buffer = std::make_shared<util::ConcurrentRingBuffer<msg::TransformStamped>>(16);
    auto vis_buffer = std::make_shared<util::ConcurrentRingBuffer<Matrix4f>>(2);

    std::mutex map_mutex;

    comm::QueueBridge<msg::TransformStamped, true> transform_bridge{transform_buffer, nullptr, config.bridge.transform_port_to()};
    comm::QueueBridge<msg::PointCloudPtrStamped, true> pointcloud_send_bridge{pointcloud_send_buffer, nullptr, config.bridge.pcl_port_to()};

    gpiod::chip button_chip(config.gpio.button_chip());
    ui::Button button{button_chip.get_line(config.gpio.button_line())};
    gpiod::chip led_chip(config.gpio.led_chip());
    ui::Led led{led_chip.get_line(config.gpio.led_line())};

    Logger::info("Application initialized!");

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
            if (err != EAGAIN)
            {
                Logger::error("Wait for signal failed (", std::strerror(err), ")! Stopping Application...");
            }
        }
        return false;
    };

    if (!config.bridge.use_from() && config.imu.use_phidgets())
    {
        Logger::info("Wait for calibration (hit ENTER)");
        led.setFrequency(1.0);
        if (!button.wait_for_press_or_condition(running_condition))
        {
            return 0;
        }
        led.setFrequency(5.0);

        Logger::info("Calibrating IMU");
        imu_driver->start();
        imu_driver->stop();
        Logger::info("Phidgets calibrated");
    }
    else
    {
        Logger::info("Using data sent from bridge");
    }

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

        imu_buffer->clear();
        imu_bridge_buffer->clear();
        pointcloud_buffer->clear();
        pointcloud_bridge_buffer->clear();
        transform_buffer->clear();
        vis_buffer->clear();
        ground_truth_buffer->clear();
        further_ground_truth_buffer->clear();

        std::ostringstream filename;
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        filename << "GlobalMap_" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S") << ".h5";
        auto global_map = std::make_shared<GlobalMap>(
                              std::filesystem::path(config.slam.map_path()) / filename.str(),
                              tau, initial_weight);

        auto local_map = std::make_shared<LocalMap>(
                             config.slam.map_size_x(),
                             config.slam.map_size_y(),
                             config.slam.map_size_z(),
                             global_map, command_queue);

        MapThread map_thread{local_map,
                             map_mutex,
                             config.slam.map_update_period(),
                             config.slam.map_update_position_threshold(),
                             config.bridge.tsdf_port_to(),
                             point_scale,
                             command_queue};
        CloudCallback cloud_callback{registration,
                                     pointcloud_bridge_buffer,
                                     local_map,
                                     global_map,
                                     transform_buffer,
                                     pointcloud_send_buffer,
                                     further_ground_truth_buffer,
                                     send_after_registration,
                                     command_queue,
                                     map_thread,
                                     map_mutex,
                                     point_scale};

        {
            Runner run_lidar_driver(*lidar_driver);
            Runner run_lidar_bridge(preprocessing);
            if (imu_driver)
            {
                Runner run_imu_driver(*imu_driver);
            }
            Runner run_imu_bridge(imu_bridge);
            Runner run_transform_bridge(transform_bridge);
            Runner run_pointcloud_send_bridge(pointcloud_send_bridge);
            Runner run_map_thread(map_thread);
            //Runner run_gt_thread(ground_truth_buffer);

           
            Runner run_ground_truth_driver(*ground_truth_driver);
            
            // clear any remaining messages FIXME: WHY IS THIS NECESSARY???
            std::this_thread::sleep_for(1s);
            imu_buffer->clear();
            imu_bridge_buffer->clear();
            pointcloud_buffer->clear();
            pointcloud_bridge_buffer->clear();
            transform_buffer->clear();
            vis_buffer->clear();
            ground_truth_buffer->clear();
            further_ground_truth_buffer->clear();

            Runner run_cloud_callback{cloud_callback};

            Logger::info("SLAM started! Running...");
            std::this_thread::sleep_for(2s);
            if (!button.wait_for_press_or_condition(running_condition))
            {
                return 0;
            }
        }
        Logger::info("SLAM stopped!");
        Logger::info("Write local and global map...");
        // save Map to Disk
        local_map->write_back();
        Logger::info("Local and global map saved!");
        local_map.reset();
        global_map.reset();
        Logger::info("Clear local and global map!");
    }
}