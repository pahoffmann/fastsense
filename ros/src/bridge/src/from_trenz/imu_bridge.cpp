/**
 * @file 
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <omp.h>
#include <algorithm>
#include <ros/ros.h>
#include <bridge/from_trenz/imu_bridge.h>

using namespace fastsense::util;
using namespace fastsense::bridge;

// TODO init covariance static?
// TODO params

ImuBridge::ImuBridge(ros::NodeHandle& n, const std::string& board_addr) 
:   BridgeBase{n, "imu_bridge/from_trenz/raw", board_addr}, 
    ProcessThread{},
    imu_ros_{},
    mag_ros_{},
    mag_pub_{},
    angular_velocity_covariance_{},
    linear_acceleration_covariance_{},
    magnetic_field_covariance_{}
{
    mag_pub_ = n.advertise<sensor_msgs::MagneticField>("imu_bridge/from_trenz/mag", 5);
    initCovariance();
}

void ImuBridge::run()
{
    while (running && ros::ok())
    {
        BridgeBase::run();
        ROS_INFO_STREAM("Received imu msg\n");
    }
}

void ImuBridge::initCovariance()
{
    double ang_vel_var = params::angular_velocity_stdev_ * params::angular_velocity_stdev_;
    double lin_acc_var = params::linear_acceleration_stdev_ * params::linear_acceleration_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                angular_velocity_covariance_[idx]    = ang_vel_var;
                linear_acceleration_covariance_[idx] = lin_acc_var;
            }
            else
            {
                angular_velocity_covariance_[idx]    = 0.0;
                linear_acceleration_covariance_[idx] = 0.0;
            }
        }
    }
    // build covariance matrix

    double mag_field_var = params::magnetic_field_stdev_ * params::magnetic_field_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                magnetic_field_covariance_[idx] = mag_field_var;
            }
            else
            {
                magnetic_field_covariance_[idx] = 0.0;
            }
        }
    }
}

void ImuBridge::convert()
{  
    const auto& [ data, timestamp ] = msg();

    auto t_since_epoch = timestamp.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(t_since_epoch).count();
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_since_epoch).count() % 1'000'000;

    imu_ros_.header.frame_id = "base_link";
    imu_ros_.header.stamp = ros::Time(sec, nsec);
    imu_ros_.orientation.x = 0;
    imu_ros_.orientation.y = 0;
    imu_ros_.orientation.z = 0;
    imu_ros_.orientation.w = 0;

    imu_ros_.angular_velocity.x = data.ang.x();
    imu_ros_.angular_velocity.y = data.ang.y();
    imu_ros_.angular_velocity.z = data.ang.z();

    imu_ros_.linear_acceleration.x = data.acc.x();
    imu_ros_.linear_acceleration.y = data.acc.y();
    imu_ros_.linear_acceleration.z = data.acc.z();

    std::copy(  linear_acceleration_covariance_.begin(),
                linear_acceleration_covariance_.end(),
                imu_ros_.linear_acceleration_covariance.begin());

    std::copy(  angular_velocity_covariance_.begin(),
                angular_velocity_covariance_.end(),
                imu_ros_.angular_velocity_covariance.begin());

    mag_ros_.header.frame_id = "base_link";
    mag_ros_.header.stamp = ros::Time(sec, nsec);
    mag_ros_.magnetic_field.x = data.mag.x();
    mag_ros_.magnetic_field.y = data.mag.y();
    mag_ros_.magnetic_field.z = data.mag.z();

    std::copy(  magnetic_field_covariance_.begin(), 
                magnetic_field_covariance_.end(), 
                mag_ros_.magnetic_field_covariance.begin());

    ROS_INFO_STREAM("Converted imu values\n");
}

void ImuBridge::publish()
{
    pub().publish(imu_ros_);
    mag_pub_.publish(mag_ros_);
    ROS_INFO_STREAM("published imu values\n");
}