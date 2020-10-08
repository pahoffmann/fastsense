/**
 * @author Marcel Flottmann
 */

#include <chrono>
#include <csignal>
#include <cstring>
#include <algorithm>
#include <iostream>

#include "Application.h"
#include <msg/imu_msg.h>
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
    : signal_set{},
      imuBuffer{std::make_shared<data::ImuStampedBuffer>(ConfigManager::config().imu.bufferSize())},
      pointcloudBuffer{std::make_shared<data::PointCloudStampedBuffer>(ConfigManager::config().lidar.bufferSize())},
      imuDriver{imuBuffer},
      lidarDriver{ConfigManager::config().lidar.port(), pointcloudBuffer}
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
    Logger::info("Starting Application...");
    Runner run_lidarDriver(lidarDriver);
    Runner run_imuDriver(imuDriver);
    Logger::info("Application started");


    Registration registration{ConfigManager::config().registration.max_iterations(), ConfigManager::config().registration.it_weight_gradient()};
    std::shared_ptr<GlobalMap> global_map_ptr = std::make_shared<GlobalMap>("GlobalMap.h5", ConfigManager::config().slam.max_distance() / ConfigManager::config().slam.map_resolution(), ConfigManager::config().slam.initial_map_weight());
    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();
    LocalMap local_map{ConfigManager::config().slam.map_size_x(), ConfigManager::config().slam.map_size_y(), ConfigManager::config().slam.map_size_z(), global_map_ptr, q};
    Matrix4f pose = Matrix4f::Identity();

    CloudCallback cloud_callback{registration, pointcloudBuffer, local_map, global_map_ptr, pose};
    Runner<CloudCallback> run_cloud_callback{cloud_callback};

    ImuCallback imu_callback{registration, imuBuffer};
    Runner<ImuCallback> run_imu_callback{imu_callback};

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