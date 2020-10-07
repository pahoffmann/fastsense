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
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <comm/sender.h>
#include <comm/bridge_messages.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::util::logging;
using namespace std::chrono_literals;

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
    // Runner run_synchronizer(synchronizer);
    // Runner run_lidarDriver(lidarDriver);
    // Runner run_imuDriver(imuDriver);
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

    std::vector<std::pair<float, float>> tsdf_values(30'000);
    std::fill(tsdf_values.begin(), tsdf_values.end(), std::make_pair(1, 1));

    msg::ImuMsg imu_msg;
    imu_msg.acc += 1;
    imu_msg.acc.z() = -9.81;
    imu_msg.ang += 0;
    imu_msg.mag += 0;

    comm::Sender<comm::TSDFBridgeMessage> tsdf_sender{"192.168.1.1", 6666};
    comm::Sender<msg::ImuMsg> imu_sender{"192.168.1.1", 5555};

    while (true) {
        // Iteration done
        // * TSDF berechnet
        // * Werte in Buffer
        // * Erstelle local map mit diesen Werten
        // auto& buffer = local_map.getBuffer();
        // std::vector<std::pair<float, float>> tsdf_values(buffer.size());
        // std::copy(buffer.begin(), buffer.end(), tsdf_values.begin());
        // comm::TSDFBridgeMessage msg { .map_resolution_ }
        // Sender.send(msg)
        comm::TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = 1;
        tsdf_msg.size_ = {10, 10, 10};
        tsdf_msg.map_resolution_ = 1;
        tsdf_msg.offset_ = {1, 1, 1};
        tsdf_msg.pos_ = {1, 1, 1};
        tsdf_msg.offset_ = {1, 1, 1};

        tsdf_sender.send(&tsdf_msg);

        imu_sender.send(&imu_msg);

        std::cout << "Sent\n";
        std::this_thread::sleep_for(1s);
    }

    return 0;
}