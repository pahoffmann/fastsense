#pragma once

/**
 * @file base_kernel.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <hw/fpga_manager.h>

namespace fastsense::kernels
{

/**
 * @brief Base class for FPGA kernels from the software perspective.
 * For every kernel there should be a derived class, that encapsulates the
 * call to that kernel.
 * 
 */
class BaseKernel
{
private:
    int narg_;
protected:
    /**
     * @brief Add argument at position narg to FPGA function call
     * 
     * @tparam T type of arg
     * @param arg argument to add to FPGA function call
     */
    template <typename T>
    inline void setArg(const T& arg)
    {
        kernel_.setArg(narg_++, arg);
    }

    /**
     * @brief Set kernel args like a function call to the kernel
     * 
     * @tparam T First parameter type
     * @tparam Args Other parameter types
     * @param arg First parameter
     * @param args Other parameters
     */
    template <typename T, typename ...Args>
    inline void setArgs(const T& arg, const Args& ...args)
    {
        setArg(arg);
        setArgs(args...);
    }

    /**
     * @brief Set kernel args like a function call to the kernel (single parameter or end of variadic template)
     * 
     * @tparam T Paramter type
     * @param arg Parameter value
     */
    template <typename T>
    inline void setArgs(const T& arg)
    {
        setArg(arg);
    }

    /// reset narg to 0
    void resetNArg()
    {
        narg_ = 0;
    }

    /// FPGA Kernel
    cl::Kernel kernel_;

    /// Pointer to Command Queue
    fastsense::CommandQueuePtr cmd_q_;

    /// List of events to wait on before execution (e.g. memory transfers)
    std::vector<cl::Event> pre_events_;

    /// List of events to wait on kernel execution
    std::vector<cl::Event> execute_events_;

    /// List of events to wait on to fully complete (e.g. memory transfers)
    std::vector<cl::Event> post_events_;
public:
    /**
     * @brief Construct a kernel object
     * 
     * @param queue Command queue to enqueue the tasks on
     * @param name Name of the kernel
     */
    inline BaseKernel(const fastsense::CommandQueuePtr& queue, const char* name)
        :   narg_{0},
            kernel_{fastsense::hw::FPGAManager::get_program(), name},
            cmd_q_{queue},
            pre_events_(1),
            execute_events_(1),
            post_events_(1)
    {}

    /// default destructor
    virtual ~BaseKernel() = default;

    /// delete copy assignment operator
    BaseKernel& operator=(const BaseKernel& other) = delete;

    /// delete move assignment operator
    BaseKernel& operator=(BaseKernel&&) noexcept = delete;

    /// delete copy constructor
    BaseKernel(const BaseKernel&) = delete;

    /// delete move constructor
    BaseKernel(BaseKernel&&) = delete;

    /// Wait until Kernel completes
    virtual void waitComplete()
    {
        cl::Event::waitForEvents(post_events_);
    }
};

} // namespace fastsense::kernels