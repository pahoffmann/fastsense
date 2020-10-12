#pragma once

/**
 * @author Marc Eisoldt
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <util/types.h>
#include <util/point_hw.h>
#include <hw/fpga_manager.h>
#include <hw/buffer/buffer.h>

#include <iostream>

struct IntTuple
{
    int first = 0;
    int second = 0;
};

namespace fastsense::kernels
{

class TSDFKernel : public BaseKernel
{
    std::unique_ptr<buffer::InputOutputBuffer<IntTuple>> new_entries;
public:
    TSDFKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_tsdf"}
    {

    }

    ~TSDFKernel() = default;

    void run(map::LocalMap& map, const buffer::InputBuffer<PointHW>& scan_points, int tau, int max_weight)
    {
        new_entries = std::make_unique<buffer::InputOutputBuffer<IntTuple>>(cmd_q_, map.get_size().x() * map.get_size().y() * map.get_size().z());

        resetNArg();
        setArg(scan_points.getBuffer());
        setArg((int)scan_points.size());
        setArg(map.getBuffer().getBuffer());
        setArg(map.get_size().x());
        setArg(map.get_size().y());
        setArg(map.get_size().z());
        setArg(map.get_pos().x());
        setArg(map.get_pos().y());
        setArg(map.get_pos().z());
        setArg(map.get_offset().x());
        setArg(map.get_offset().y());
        setArg(map.get_offset().z());
        setArg(new_entries->getBuffer());
        setArg(tau);
        setArg(max_weight);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan_points.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels