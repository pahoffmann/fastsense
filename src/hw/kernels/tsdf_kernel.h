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
    int x = 0;
    int y = 0;
    int z = 0;
};

struct IntTuple
{
    int first = 0;
    int second = 0;
};

namespace fastsense::kernels
{

class TSDFKernel : public BaseKernel
{

public:
    TSDFKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_tsdf"}
    {

    }

    ~TSDFKernel() = default;

    void run(map::LocalMap& map, const buffer::InputBuffer<Point>& scan_points, int tau, int max_weight)
    {
        resetNArg();

        setArg(scan_points.getBuffer());

        // auto queue = fastsense::hw::FPGAManager::create_command_queue();
        // buffer::InputOutputBuffer<Point> point_data(queue, scan_points.size());

        // for(size_t i = 0; i < scan_points.size(); ++i)
        // {
        //     point_data[i].x = scan_points[i].x();
        //     point_data[i].y = scan_points[i].y();
        //     point_data[i].z = scan_points[i].z();
        // }

        // setArg(point_data.getBuffer());
        // setArg(static_cast<int>(scan_points.size()));

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

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        buffer::InputOutputBuffer<IntTuple> new_entries(queue, m.sizeX * m.sizeY * m.sizeZ);
        setArg(new_entries.getBuffer());

        setArg(tau);
        setArg(max_weight);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan_points.getBuffer(), new_entries.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels