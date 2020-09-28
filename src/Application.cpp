/**
 * @file Application.cpp
 * @author Marcel Flottmann
 * @date 2020-09-15
 */

#include "Application.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <csignal>
#include <cstring>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;

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
      syncedDataBuffer{std::make_shared<data::SyncedDataBuffer>(ConfigManager::config().sensorSync.bufferSize())},
      imuDriver{imuBuffer},
      lidarDriver{ConfigManager::config().lidar.port(), pointcloudBuffer},
      synchronizer{imuBuffer, pointcloudBuffer, syncedDataBuffer}
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
    Runner run_synchronizer(synchronizer);
    Runner run_lidarDriver(lidarDriver);
    Runner run_imuDriver(imuDriver);
    Logger::info("Application started");

    int sig;
    int ret = sigwait(&signal_set, &sig);
    if (ret == -1)
    {
        auto err = errno;
        Logger::fatal("Wait for signal failed (", std::strerror(err), ")! Stopping Application...");
        return -1;
    }

    Logger::info("Stopping Application...");

    while (true) {
        // Iteration done
        // * TSDF berechnet
        // * Werte in Buffer
        // * Erstelle local map mit diesen Werten
        auto& buffer = local_map.getBuffer();
        std::array<std::pair<float, float>> tsdf_values(buffer.size());
        std::copy(buffer.begin(), buffer.end(), tsdf_values.begin());
    }

    return 0;
}