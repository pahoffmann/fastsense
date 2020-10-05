#pragma once

/**
 * @author Patrick Hoffmann
 */

#include <iostream>

#include <hw/buffer/buffer.h>
#include <msg/point_cloud.h>
#include <msg/quaternion.h>
#include "base_kernel.h"

namespace fastsense::kernels
{

class PclTransformKernel : public BaseKernel
{
public:
    PclTransformKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_vadd")
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
        cmd_q_->enqueueMigrateMemObjects({pcl_in, inbuffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({pcl_out}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
}

} // namespace fastsense::kernels