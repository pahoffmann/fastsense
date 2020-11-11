#pragma once

/**
 * @file reg_kernel.h
 * @author Patrick Hoffmann
 */

#include <hw/kernels/base_kernel.h>
#include <map/local_map.h>
#include <util/point_hw.h>

namespace fastsense::kernels
{

class RegistrationKernel : public BaseKernel
{
public:
    RegistrationKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_reg"}
    {}

    ~RegistrationKernel() = default;

    /**
     * @brief interface between the software and the hw, calls the run method of the kernel, writes all the data coming from the kernel
     *        into the datatypes used by the software
     *
     * @param map           current local map
     * @param scan_points   points from the current velodyne scan
     * @param local_h       reference for the local h matrix used by the software
     * @param local_g       reference for the local g matrix used by the software
     * @param local_error   local error ref
     * @param local_count   local count ref
     * @param transform     transform from last registration iteration (including imu one) - needs to be applied in the kernel
     */
    void synchronized_run(map::LocalMap& map, buffer::InputBuffer<PointHW>& point_data, int max_iterations, float it_weight_gradient, Eigen::Matrix4f& transform)
    {
        buffer::OutputBuffer<float> out_transform(cmd_q_, 16); //matrix buffer for g matrix TODO: determine if this needs to be in registration.cpp

        buffer::InputBuffer<float> in_transform(cmd_q_, 16);

        //write last transform to buffer
        for (int i = 0; i < 16; i++)
        {
            in_transform[i] = transform[i];
        }

        // std::cout << "Transform matrix sw kernel:\n" << transform << std::endl;

        //run the encapsulated kernel
        run(map, point_data, max_iterations, it_weight_gradient, in_transform, out_transform);

        waitComplete();

        for (int i = 0; i < 16; i++)
        {
            transform[i] = out_transform[i];
        }
    }

    /**
     * @brief Called by the synchronized run method, uses the kernel to register
     *
     * @param map
     * @param scan_points
     * @param outbuf
     * @param queue
     */
    void run(map::LocalMap& map, buffer::InputBuffer<PointHW>& point_data, int max_iterations, float it_weight_gradient,
             buffer::InputBuffer<float>& in_transform, buffer::OutputBuffer<float>& out_transform)
    {
        resetNArg();

        //std::cout << "Point data: size: " << static_cast<int>(point_data.size()) << std::endl << std::endl;

        auto m = map.get_hardware_representation();

        setArg(point_data.getBuffer());
        setArg(static_cast<int>(point_data.size()));
        setArg(map.getBuffer().getBuffer());
        setArg(map.getBuffer().getBuffer());
        setArg(map.getBuffer().getBuffer());
        setArg(m.sizeX);
        setArg(m.sizeY);
        setArg(m.sizeZ);
        setArg(m.posX);
        setArg(m.posY);
        setArg(m.posZ);
        setArg(m.offsetX);
        setArg(m.offsetY);
        setArg(m.offsetZ);
        setArg(max_iterations);
        setArg(it_weight_gradient);
        setArg(in_transform.getBuffer());
        setArg(out_transform.getBuffer());

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), point_data.getBuffer(), in_transform.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({out_transform.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels