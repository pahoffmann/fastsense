#pragma once

/**
 * @file registration.h
 * @author Patrick Hoffmann
 * @author Adrian Nitschmann
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 * @author Marc Eisoldt
 * @brief
 * @version 0.1
 * @date 2020-08-24
 *
 * @copyright Copyright (c) 2020
 *
 */


// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Imu.h>
// #include <ros/time.h>

//#include <prototyping/REGConfig.h>

//#include <prototyping/ring_buffer/ring_buffer.h>
//#include <prototyping/types.h>

#include <map/local_map.h>
#include <util/types.h>
#include <util/time_stamp.h>
#include <msg/msgs_stamped.h>

#include <cmath>
#include <mutex>
#include <chrono>
#include <utility>
#include <algorithm>


#ifndef REGISTRATION_H_
#define REGISTRATION_H_

namespace fastsense::registration
{

/**
 * @brief
 *
 */
class Registration
{

private:
    typedef Eigen::Matrix<float, 6, 1> Matrix6x1; //jacobi, tf
    typedef Eigen::Matrix<float, 6, 3> Matrix6x3; //first part of jacobi
    typedef Eigen::Matrix<float, 3, 1> Matrix3x1; //second part of jacobi
    typedef Eigen::Matrix<float, 6, 6> Matrix6x6; //second part of jacobi
    typedef Eigen::Matrix<float, 4, 4> Matrix4x4; //transform matrix
    typedef Eigen::Matrix<float, 3, 3> Matrix3x3; //rotation matrix

    struct Point
    {
        float x;
        float y;
        float z;
        char fill[4];
        float intensity;
        short ring;
        char fill2[10];
    };

    int max_iterations_;
    double weighting_constant_;
    double it_weight_offset_;
    double it_weight_gradient_;

    std::mutex mutex_;
    Matrix4x4 global_transform_; //used to store the transform since the last registration (right now calculated using the angular velocities by the IMU)
    fastsense::util::TimeStamp imu_time_;
    bool first_imu_msg_;
    
    /**
     * @brief transforms xi vector 6x1 (angular and linear velocity) to transformation matrix 4x4
     *
     * @param xi vector
     */
    Matrix4x4 xi_to_transform(Matrix6x1 xi);

    static inline float filter_value(const std::pair<float, float>& buf_entry);

    static inline float interpolate(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, const Vector3& point);

    static inline float interpolate(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, const Vector3& point, int buf_x, int buf_y, int buf_z);

public:

    /**
     * @brief Construct a new Registration object, used to register a pointcloud with the current ring buffer
     *
     */
    Registration(unsigned int max_iterations = 500, double weighting_constant = 100.0, double it_weight_offset = 0.0, double it_weight_gradient = 0.01) : 
    max_iterations_(max_iterations),
    weighting_constant_(weighting_constant),
    it_weight_offset_(it_weight_offset),
    it_weight_gradient_(it_weight_gradient),
    first_imu_msg_(true)
    {
        global_transform_.setIdentity();
    }

    /**
     * Destructor of the ring buffer.
     * Deletes the array in particular.
     */
    virtual ~Registration();

    float calc_weight(float x) const 
    {
        auto value = fabs(x);

        if(value <= weighting_constant_)
        {
            return 1.0;
        }

        return weighting_constant_ / value;
    }

    /**
     * @brief Registers the given pointcloud with the local ring buffer. Transforms the cloud
     *
     * @param cur_buffer
     * @param cloud
     * @return Matrix4x4
     */
    Matrix4x4 register_cloud(const fastsense::map::LocalMap<std::pair<float, float>>& localmap, ScanPoints_t<Vector3>& cloud);

    /**
     * @brief Updates the IMU data used by the registration method
     *
     * @param imu
     */
    void update_imu_data(const fastsense::msg::ImuMsgStamped& imu);

    /**
     * @brief Transforms a given pointcloud with the transform and stores it inside the out_cloud
     *
     * @param in_cloud
     * @param out_cloud
     * @param transform
     */
    static void transform_point_cloud(ScanPoints_t<Vector3>& in_cloud, const Matrix4x4& transform);
};


} //namespace fastsense

#endif
