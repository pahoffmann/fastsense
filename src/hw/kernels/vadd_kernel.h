#pragma once

#include "base_kernel.h"

#include <hw/buffer/buffer.h>
#include <hw/fpga_program.h>
#include <iostream>

namespace fastsense::kernels {

class VaddKernel : public BaseKernel
{
public:
    VaddKernel(CommandQueuePtr queue, const hw::FPGAProgram& program) : BaseKernel(queue, program.getProgram(), "krnl_vadd") {}
    ~VaddKernel() = default;

    void run(buffer::InputBuffer<int>& inbuffer_a, buffer::InputBuffer<int>& inbuffer_b, buffer::OutputBuffer<int>& outbuf, int data_size)
    {
        resetNArg();
        setArg(inbuffer_a.getBuffer());
        setArg(inbuffer_b.getBuffer());
        setArg(outbuf.getBuffer());
        setArg(data_size);

        std::cout << "data to kernel\n";
        cmd_q_->enqueueMigrateMemObjects({inbuffer_a.getBuffer(), inbuffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE/* 0 means from host*/);

        //Launch the Kernel
        std::cout << "launch kernel\n";
        cmd_q_->enqueueTask(kernel_);
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST);
        cmd_q_->finish();
    }
};

} // namespace fastsense::kernels