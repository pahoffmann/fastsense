#pragma once

/**
 * @file streamin_kernel.h
 * @author Marcel Flottmann
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>

namespace fastsense::kernels
{

class StreamInKernel : public BaseKernel
{
public:
    StreamInKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_streamin")
    {}

    ~StreamInKernel() = default;

    void run(buffer::OutputBuffer<int>& out)
    {
        resetNArg();
        setArg(out.getBuffer());

        // Write buffers
        // No buffers

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({out.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels
