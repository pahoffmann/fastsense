#pragma once

/**
 * @file registration.h
 * @author Patrick Hoffmann
 * @author Adrian Nitschmann
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <mutex>
#include <algorithm>

#include "imu_accumulator.h"
#include <msg/imu.h>
#include <hw/kernels/reg_kernel.h>

namespace fastsense::registration
{

/**
 * @brief
 *
 */
class Registration
{
    using Matrix6i = Eigen::Matrix<long, 6, 6>;
    using Matrix6f = Eigen::Matrix<float, 6, 6>;
    using Vector6i = Eigen::Matrix<long, 6, 1>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;

private:
    size_t max_iterations_;
    float it_weight_gradient_;

    std::mutex mutex_;
    ImuAccumulator imu_accumulator_;


    fastsense::kernels::RegistrationKernel krnl;

    /**
     * @brief transforms xi vector 6x1 (angular and linear velocity) to transformation matrix 4x4
     *
     * @param xi vector
     */
    Matrix4f xi_to_transform(Vector6f xi);

public:

    /**
     * @brief Construct a new Registration object, used to register a pointcloud with the current ring buffer
     *
     */
    Registration(const fastsense::CommandQueuePtr& q, size_t max_iterations = 50, float it_weight_gradient = 0.0);

    /**
     * Destructor of the registration.
     */
    ~Registration() = default;

    /**
     * @brief Registers the given pointcloud with the local ring buffer. Transforms the cloud
     *
     * @param cur_buffer
     * @param cloud
     * @return Matrix4f
     */
    Matrix4f register_cloud(fastsense::map::LocalMap& localmap, fastsense::buffer::InputBuffer<PointHW>& cloud);

    /**
     * @brief Updates the IMU data used by the registration method
     *
     * @param imu
     */
    void update_imu_data(const fastsense::msg::ImuStamped& imu);

    /**
     * @brief Transforms a given pointcloud with the transform
     *
     * @param in_cloud
     * @param transform
     */
    static void transform_point_cloud(ScanPoints_t& in_cloud, const Matrix4f& transform);

    /**
     * @brief Transforms a given pointcloud with the transform
     *
     * @param in_cloud
     * @param transform
     */
    static void transform_point_cloud(fastsense::buffer::InputBuffer<PointHW>& in_cloud, const Matrix4f& transform);
};


} //namespace fastsense
