/**
 * @file hw_buffer.cpp
 * @author Julian Gaal
 * @date 2020-09-28
 */

#include "catch2_config.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <data/sensor_sync.h>
#include <msg/imu_msg.h>

using namespace fastsense;
using namespace fastsense::util::config;
using namespace fastsense::msg;

TEST_CASE("Test Buffer STL Container", "[]")
{
    std::cout << "Testing 'Test Buffer STL Container'" << std::endl;   
    constexpr size_t buffer_size = 5;
    data::ImuStampedBufferPtr imuBuffer{std::make_shared<data::ImuStampedBuffer>(buffer_size)};
    
    for (size_t i = 0; i < buffer_size*2; i++)
    {
        ImuMsg imu_msg{};
        imu_msg.acc += 1;
        imu_msg.ang += 2;
        imu_msg.mag += 3;
    }

    SECTION("Test #1")
    {
        std::cout << "    Section 'Test #1'" << std::endl;

    }
}
