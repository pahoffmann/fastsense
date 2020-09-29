/**
 * @file vadd_kernel.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-9
 */

#pragma once

#include "base_kernel.h"
#include <map/local_map.h>
#include <hw/buffer/buffer.h>
#include <iostream>
#include <msg/point.h>

namespace fastsense::kernels
{

class RegKernel : public BaseKernel
{
    std::vector<cl::Event> preEvents;
    std::vector<cl::Event> executeEvents;
    std::vector<cl::Event> postEvents;
public:
    RegKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_reg"),
          preEvents(1),
          executeEvents(1),
          postEvents(1)
    {}
    ~RegKernel() = default;


    //todo: determine parameters for the kernel.
    // localmap - marcel? - pointcloud
    void run(map::LocalMap<std::pair<int, int>>& map, buffer::InputBuffer<fastsense::msg::Point>& scan, buffer::OutputBuffer<int>& outbuf, int data_size)
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
        setArg(scan.getBuffer());
        setArg(outbuf.getBuffer());
        setArg(data_size);


        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &preEvents[0]);

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