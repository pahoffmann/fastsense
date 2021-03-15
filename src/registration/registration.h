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

private:
    size_t max_iterations_;      // maxmimum number of error function minimizations
    float it_weight_gradient_;   // linear weighted gradient to weigh tranforms in futher iterations less than in earlier to avoid jumping over the optimium
    float epsilon_;              // epsilon value used for the termination condition

    // used to accumulate the incoming IMU data until the laserscan data comes in
    ImuAccumulator imu_accumulator_;
    // Registration Kernel object (reg_kernel.h) - softwarepart of the kernel
    fastsense::kernels::RegistrationKernel krnl;

public:

    /**
     * @brief Construct a new Registration object
     *
     * TODO
     *
     * @param q xilinx command queue
     * @param buffer imu buffer stamped shared ptr
     * @param max_iterations max convergence iterations
     * @param it_weight_gradient learning rate weight gradient
     */
    Registration(fastsense::CommandQueuePtr q, msg::ImuStampedBuffer::Ptr& buffer, unsigned int max_iterations = 50, float it_weight_gradient = 0.0, float epsilon = 0.01);

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
    void register_cloud(fastsense::map::LocalMap& localmap,
                        fastsense::buffer::InputBuffer<PointHW>& cloud,
                        const util::HighResTimePoint& cloud_timestamp,
                        Matrix4f& pose);

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
