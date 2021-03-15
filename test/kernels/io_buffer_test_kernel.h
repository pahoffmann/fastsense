#pragma once

/**
 * @file io_buffer_test_kernel.h
 * @author Julian Gaal
 *
 * Test
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>

namespace fastsense::kernels
{

class IOBufferTestKernel : public BaseKernel
{
public:
    IOBufferTestKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_io_buffer_test"}
    {}

    ~IOBufferTestKernel() = default;

    /**
     * @brief used to synchronize the kernel run and encapsulate the functionality of waitcomplete inside the kernel
     * 
     * @param data 
     * @param size 
     */
    void synchronized_run(buffer::InputOutputBuffer<int>& data, int size){
        // run the kernel
        run(data, size);
        
        //wait for its completion
        waitComplete();
    }
    
    /**
     * @brief runs the test kernel
     * 
     * @param data 
     * @param size 
     */
    void run(buffer::InputOutputBuffer<int>& data, int size)
    {
        resetNArg();
        setArg(data.getBuffer());
        setArg(size);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({data.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({data.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels
