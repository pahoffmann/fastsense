#pragma once

/**
 * @file config.h
 * @author Marcel Flottmann
 */

#include "config_types.h"

namespace fastsense::util::config
{

struct ImuConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
    DECLARE_CONFIG_ENTRY(size_t, filterSize);
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

struct BridgeConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, use_from);
    DECLARE_CONFIG_ENTRY(size_t, use_to);

    DECLARE_CONFIG_ENTRY(std::string, host_from);

    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_from);
    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_to);

    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_from);
    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_to);

    DECLARE_CONFIG_ENTRY(uint16_t, transform_port_to);
    DECLARE_CONFIG_ENTRY(uint16_t, tsdf_port_to);
};


struct RegistrationConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(unsigned int, max_iterations);
    DECLARE_CONFIG_ENTRY(float, it_weight_gradient);
    DECLARE_CONFIG_ENTRY(float, epsilon);
};

struct SlamConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(float, max_distance);
    DECLARE_CONFIG_ENTRY(float, map_resolution);
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_x);
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_y);
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_z);
    DECLARE_CONFIG_ENTRY(int, max_weight);
    DECLARE_CONFIG_ENTRY(float, initial_map_weight);

    DECLARE_CONFIG_ENTRY(unsigned int, map_update_period);
    DECLARE_CONFIG_ENTRY(float, map_update_position_threshold);

};

struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(ImuConfig, imu);
    DECLARE_CONFIG_GROUP(LidarConfig, lidar);
    DECLARE_CONFIG_GROUP(SensorSyncConfig, sensorSync);
    DECLARE_CONFIG_GROUP(RegistrationConfig, registration);
    DECLARE_CONFIG_GROUP(SlamConfig, slam);
    DECLARE_CONFIG_GROUP(BridgeConfig, bridge);
};

} // namespace fastsense::util::config
