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
public:
    Application();
    ~Application() = default;

    int run();
};

} // namespace fastsense