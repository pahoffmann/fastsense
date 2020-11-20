#pragma once

/**
 * @file streamin_kernel.h
 * @author Marcel Flottmann
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>

namespace fastsense::kernels
{

class StreamOutKernel : public BaseKernel
{
public:
    StreamOutKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_streamin")
    {}

    ~StreamOutKernel() = default;

    void run(buffer::InputBuffer<int>& in)
    {
        resetNArg();
        setArg(in.getBuffer());

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({in.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        // No Buffers
    }
};

} // namespace fastsense::kernels
