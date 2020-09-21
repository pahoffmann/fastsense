/**
 * @file pcl_transform_kernel.h
 * @author Patrick Hoffmann
 * @date 2020-09-9
 */

#pragma once

#include "base_kernel.h"

#include <hw/buffer/buffer.h>
#include <msg/point_cloud.h>
#include <msg/quaternion.h>
#include <iostream>

namespace fastsense::kernels
{

    class PclTransformKernel : public BaseKernel
    {
        std::vector<cl::Event> preEvents;
        std::vector<cl::Event> executeEvents;
        std::vector<cl::Event> postEvents;
    public:
        PclTransformKernel(const CommandQueuePtr& queue)
            : BaseKernel(queue, "krnl_vadd"),
            preEvents(1),
            executeEvents(1),
            postEvents(1)
        {}
        ~PclTransformKernel() = default;


        //todo: determine parameters for the kernel, possibly a transform and a pcl
        // second parameter needs to be looked up - currently no transform msg, needs to be added ??
        void run(Pointcloud::ptr pcl_in, buffer::InputBuffer<int>& inbuffer_b, Pointcloud::ptr pcl_out, int data_size)
        {
            resetNArg();
            setArg(pcl_in);
            setArg(inbuffer_b.getBuffer());
            setArg(pcl_out);
            setArg(data_size);

            // Write buffers
            cmd_q_->enqueueMigrateMemObjects({pcl_in, inbuffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &preEvents[0]);

            // Launch the Kernel
            cmd_q_->enqueueTask(kernel_, &preEvents, &executeEvents[0]);

            // Read buffers
            cmd_q_->enqueueMigrateMemObjects({pcl_out}, CL_MIGRATE_MEM_OBJECT_HOST, &executeEvents, &postEvents[0]);
        }

        void waitComplete()
        {
            cl::Event::waitForEvents(postEvents);
        }
    }

} // namespace fastsense::kernels