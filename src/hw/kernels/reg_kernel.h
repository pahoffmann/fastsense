#pragma once

/**
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <iostream>

#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <msg/point.h>
#include "base_kernel.h"

namespace fastsense::kernels
{

class RegKernel : public BaseKernel
{
public:
    RegKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_reg")
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
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels