#pragma once

/**
 * @file config.h
 * @author Marcel Flottmann
 */

#include "config_types.h"

namespace fastsense::util::config
{

/**
 * @brief Configuration group for the IMU
 * 
 */
struct ImuConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// Size of the Buffer for incoming values
    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
    /// Size of the Window for the SlidingWindowFilter
    DECLARE_CONFIG_ENTRY(size_t, filterSize);
};

/**
 * @brief Configuration group for the LIDAR
 * 
 */
struct LidarConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// Size of the Buffer for incoming values
    DECLARE_CONFIG_ENTRY(size_t, bufferSize);
    /// The Port to listen to
    DECLARE_CONFIG_ENTRY(uint16_t, port);
    /// A Factor to apply to the entire Cloud
    DECLARE_CONFIG_ENTRY(float, pointScale);
    /// The number of rings that the lidar has
    DECLARE_CONFIG_ENTRY(int, rings);
    /// The field of view in vertical direction in degrees
    DECLARE_CONFIG_ENTRY(float, vertical_fov_angle);
};

/**
 * @brief Configuration group for the bridge
 * 
 */
struct BridgeConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// true: take input from PC, false: use Sensors
    DECLARE_CONFIG_ENTRY(bool, use_from);
    /// true: send output to PC
    DECLARE_CONFIG_ENTRY(bool, use_to);

    /// send PointCloud before Preprocessing. Only one send_* option can be active
    DECLARE_CONFIG_ENTRY(bool, send_original);
    /// send PointCloud after Preprocessing. Only one send_* option can be active
    DECLARE_CONFIG_ENTRY(bool, send_preprocessed);
    /// send PointCloud after Registration. Only one send_* option can be active
    DECLARE_CONFIG_ENTRY(bool, send_after_registration);

    /// IP Address of the PC when 'use_from' is true
    DECLARE_CONFIG_ENTRY(std::string, host_from);
    /// Timeout for the receiver
    DECLARE_CONFIG_ENTRY(uint16_t, recv_timeout);

    /// Port of the from bridge for imu
    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_from);
    /// Port of the to bridge for imu
    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_to);

    /// Port of the from bridge for pcl
    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_from);
    /// Port of the to bridge for pcl
    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_to);

    /// Port of the to bridge for transform
    DECLARE_CONFIG_ENTRY(uint16_t, transform_port_to);
    /// Port of the to bridge for tsdf
    DECLARE_CONFIG_ENTRY(uint16_t, tsdf_port_to);
};

/**
 * @brief Configuration group for the GPIO
 * 
 */
struct GPIOConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// GPIO chip of the button, e.g. gpiochip0
    DECLARE_CONFIG_ENTRY(std::string, button_chip);
    /// GPIO chip of the LED, e.g. gpiochip0
    DECLARE_CONFIG_ENTRY(std::string, led_chip);
    /// GPIO line (pin) of the button
    DECLARE_CONFIG_ENTRY(unsigned int, button_line);
    /// GPIO line (pin) of the LED
    DECLARE_CONFIG_ENTRY(unsigned int, led_line);
};

/**
 * @brief Configuration group for the registration
 * 
 */
struct RegistrationConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// Maximum number of iterations for the Registration
    DECLARE_CONFIG_ENTRY(unsigned int, max_iterations);
    /// Factor to reduce Registration influence on later iterations
    DECLARE_CONFIG_ENTRY(float, it_weight_gradient);
    /// Minimum change between two iterations to stop Registration
    DECLARE_CONFIG_ENTRY(float, epsilon);
};

/**
 * @brief Configuration group for the SLAM parameters
 * 
 */
struct SlamConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// The truncation distance in mm
    DECLARE_CONFIG_ENTRY(int, max_distance);
    /// The size of one TSDF cell in mm
    DECLARE_CONFIG_ENTRY(unsigned int, map_resolution);
    /// The number of TSDF cells in x direction
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_x);
    /// The number of TSDF cells in y direction
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_y);
    /// The number of TSDF cells in z direction
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_z);
    /// The maximum weight as a float where 1.0
    DECLARE_CONFIG_ENTRY(float, max_weight);
    /// The initial weight as a float where 1.0
    DECLARE_CONFIG_ENTRY(float, initial_map_weight);

    /// Number of Scans before a TSDF Update happens
    DECLARE_CONFIG_ENTRY(unsigned int, map_update_period);
    /// Distance since the last TSDF Update before a new one happens
    DECLARE_CONFIG_ENTRY(float, map_update_position_threshold);

    /// Path where the global map should be saved
    DECLARE_CONFIG_ENTRY(std::string, map_path);
};

/**
 * @brief Main configuration definition
 * 
 */
struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    /// IMU configuration
    DECLARE_CONFIG_GROUP(ImuConfig, imu);
    /// LIDAR configuration
    DECLARE_CONFIG_GROUP(LidarConfig, lidar);
    /// Regsitration configuration
    DECLARE_CONFIG_GROUP(RegistrationConfig, registration);
    /// GPIO configuration
    DECLARE_CONFIG_GROUP(GPIOConfig, gpio);
    /// Bridge configuration
    DECLARE_CONFIG_GROUP(BridgeConfig, bridge);
    /// SLAM parameter configuration
    DECLARE_CONFIG_GROUP(SlamConfig, slam);
};

} // namespace fastsense::util::config
