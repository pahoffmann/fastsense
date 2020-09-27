/**
 * @file local_map_test_kernel.h
 * @author Marcel Flottmann
 * @date 2020-09-16
 */

#pragma once

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <iostream>

namespace fastsense::kernels
{

class LocalMapTestKernel : public BaseKernel
{
    std::vector<cl::Event> preEvents;
    std::vector<cl::Event> executeEvents;
    std::vector<cl::Event> postEvents;
public:
    LocalMapTestKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_local_map_test"},
          preEvents{1},
          executeEvents{1},
          postEvents{1}
    {}
    ~LocalMapTestKernel() = default;

    void run(map::LocalMap<std::pair<float, float>>& map)
    {
        resetNArg();
        setArg(map.getBuffer().getBuffer());
        auto m = map.getHardwareRepresentation();
        setArg(m.sizeX);
        setArg(m.sizeY);
        setArg(m.sizeZ);
        setArg(m.posX);
        setArg(m.posY);
        setArg(m.posZ);
        setArg(m.offsetX);
        setArg(m.offsetY);
        setArg(m.offsetZ);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &preEvents[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &preEvents, &executeEvents[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &executeEvents, &postEvents[0]);
    }

    void waitComplete()
    {
        cl::Event::waitForEvents(postEvents);
    }
};

} // namespace fastsense::kernels