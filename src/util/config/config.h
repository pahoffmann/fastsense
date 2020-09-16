/**
 * @file config.h
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include "config_types.h"

namespace fastsense::util::config
{

struct ImuConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
};

struct LidarConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
    DECLARE_CONFIG_ENTRY(uint16_t, port);
};

struct SensorSyncConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
};

struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(ImuConfig, imu);
    DECLARE_CONFIG_GROUP(LidarConfig, lidar);
    DECLARE_CONFIG_GROUP(SensorSyncConfig, sensorSync);
};

}
