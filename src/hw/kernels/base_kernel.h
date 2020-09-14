/**
 * @file base_kernel.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-9
 */
#pragma once

#include <hw/opencl.h>
#include <hw/types.h>
#include <hw/fpga_manager.h>

namespace fastsense::kernels
{

class BaseKernel
{
private:
    int narg_;
protected:
    template <typename T>
    inline void setArg(const T& arg)
    {
        kernel_.setArg(narg_++, arg);
    }

    void resetNArg()
    {
        narg_ = 0;
    }

    cl::Kernel kernel_;
    fastsense::CommandQueuePtr cmd_q_;
public:
    inline BaseKernel(const fastsense::CommandQueuePtr& queue, const char* name)
        :   narg_{0},
            kernel_{fastsense::hw::FPGAManager::getProgram(), name},
            cmd_q_{queue} {}

    virtual ~BaseKernel() = default;
};

} // namespace fastsense::kernels