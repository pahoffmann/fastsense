#pragma once

/**
 * @author Marc Eisoldt
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <util/types.h>
#include <hw/fpga_manager.h>
#include <hw/buffer/buffer.h>

#include <iostream>

struct Point
{
    int x;
    int y;
    int z;
};

namespace fastsense::kernels
{

class TSDFKernel : public BaseKernel
{
public:
    TSDFKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_tsdf"}
    {}

    ~TSDFKernel() = default;

    void run(map::LocalMap& map, ScanPoints_t scan_points)
    {
        resetNArg();

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        buffer::InputOutputBuffer<Point> point_data(queue, scan_points.size());

        for(size_t i = 0; i < scan_points.size(); ++i)
        {
            point_data[i].x = scan_points[i].x();
            point_data[i].y = scan_points[i].y();
            point_data[i].z = scan_points[i].z();
        }

        setArg(point_data.getBuffer());
        setArg(static_cast<int>(scan_points.size()));

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
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels