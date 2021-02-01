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

    DECLARE_CONFIG_ENTRY(size_t, bufferSize, "Size of the Buffer for incoming values");
    DECLARE_CONFIG_ENTRY(size_t, filterSize, "Size of the Window for the SlidingWindowFilter");
};

struct LidarConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize, "Size of the Buffer for incoming values");
    DECLARE_CONFIG_ENTRY(uint16_t, port, "The Port to listen to");
};

struct BridgeConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(bool, use_from, "true: take input from PC, false: use Sensors");
    DECLARE_CONFIG_ENTRY(bool, use_to, "true: send output to PC");

    DECLARE_CONFIG_ENTRY(std::string, host_from, "IP Address of the PC when 'use_from' is true");

    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_from, "Port of the from bridge for imu");
    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_to, "Port of the to bridge for imu");

    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_from, "Port of the from bridge for pcl");
    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_to, "Port of the to bridge for pcl");

    DECLARE_CONFIG_ENTRY(uint16_t, transform_port_to, "Port of the to bridge for transform");
    DECLARE_CONFIG_ENTRY(uint16_t, tsdf_port_to, "Port of the to bridge for tsdf");
};


struct RegistrationConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(unsigned int, max_iterations, "Maximum number of iterations for the Registration");
    DECLARE_CONFIG_ENTRY(float, it_weight_gradient, "Factor to reduce Registration influence on later iterations");
    DECLARE_CONFIG_ENTRY(float, epsilon, "Minimum change between two iterations to stop Registration");
};

struct SlamConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(int, max_distance, "The truncation distance in mm");
    DECLARE_CONFIG_ENTRY(unsigned int, map_resolution, "The size of one TSDF cell in mm");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_x, "The number of TSDF cells in x direction");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_y, "The number of TSDF cells in y direction");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_z, "The number of TSDF cells in z direction");
    DECLARE_CONFIG_ENTRY(float, max_weight, "The maximum weight as a float where 1.0");
    DECLARE_CONFIG_ENTRY(float, initial_map_weight, "The initial weight as a float where 1.0");

    DECLARE_CONFIG_ENTRY(unsigned int, map_update_period, "Number of Scans before a TSDF Update happens");
    DECLARE_CONFIG_ENTRY(float, map_update_position_threshold, "Distance since the last TSDF Update before a new one happens");

};

struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(ImuConfig, imu);
    DECLARE_CONFIG_GROUP(LidarConfig, lidar);
    DECLARE_CONFIG_GROUP(RegistrationConfig, registration);
    DECLARE_CONFIG_GROUP(SlamConfig, slam);
    DECLARE_CONFIG_GROUP(BridgeConfig, bridge);
};

} // namespace fastsense::util::config
