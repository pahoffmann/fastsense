/**
 * @file vadd_kernel.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-9
 */

#pragma once

#include "base_kernel.h"

#include <hw/buffer/buffer.h>
#include <iostream>

namespace fastsense::kernels
{

class VaddKernel : public BaseKernel
{
    std::vector<cl::Event> preEvents;
    std::vector<cl::Event> executeEvents;
    std::vector<cl::Event> postEvents;
public:
    VaddKernel(const CommandQueuePtr& queue, const cl::Program program)
        : BaseKernel(queue, program, "krnl_vadd"),
          preEvents(1),
          executeEvents(1),
          postEvents(1)
    {}
    ~VaddKernel() = default;

    void run(buffer::InputBuffer<int>& inbuffer_a, buffer::InputBuffer<int>& inbuffer_b, buffer::OutputBuffer<int>& outbuf, int data_size)
    {
        resetNArg();
        setArg(inbuffer_a.getBuffer());
        setArg(inbuffer_b.getBuffer());
        setArg(outbuf.getBuffer());
        setArg(data_size);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({inbuffer_a.getBuffer(), inbuffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &preEvents[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &preEvents, &executeEvents[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &executeEvents, &postEvents[0]);
    }

    void waitComplete()
    {
        cl::Event::waitForEvents(postEvents);
    }
};

} // namespace fastsense::kernels