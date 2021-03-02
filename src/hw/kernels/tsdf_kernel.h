#pragma once

/**
 * @file tsdf_kernel.h
 * @author Marc Eisoldt
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <util/point_hw.h>
#include <hw/buffer/buffer.h>

#include <iostream>

namespace fastsense::kernels
{

class TSDFKernel : public BaseKernel
{
    buffer::InputOutputBuffer<TSDFValue> new_entries;

public:

    TSDFKernel(const CommandQueuePtr& queue, size_t map_size)
        : BaseKernel{queue, "krnl_tsdf"}, new_entries{cmd_q_, map_size}
    {

    }

    ~TSDFKernel() override = default;

    /// delete copy assignment operator
    TSDFKernel& operator=(const TSDFKernel& other) = delete;

    /// delete move assignment operator
    TSDFKernel& operator=(TSDFKernel&&) noexcept = delete;

    /// delete copy constructor
    TSDFKernel(const TSDFKernel&) = delete;

    /// delete move constructor
    TSDFKernel(TSDFKernel&&) = delete;

    void run(map::LocalMap& map, const buffer::InputBuffer<PointHW>& scan_points, TSDFValue::ValueType tau, TSDFValue::WeightType max_weight, PointHW up = PointHW(0, 0, MATRIX_RESOLUTION))
    {
        for (int i = 0; i < (int)new_entries.size(); ++i)
        {
            new_entries[i] = TSDFValue(0, 0);
        }

        constexpr int SPLIT_FACTOR = 4;

        resetNArg();
        for (int i = 0; i < SPLIT_FACTOR; i++)
        {
            setArg(scan_points.getBuffer());
        }
        setArg((int)scan_points.size());
        for (int i = 0; i < SPLIT_FACTOR; i++)
        {
            setArg(map.getBuffer().getBuffer());
        }

        setArg(map.get_size().x());
        setArg(map.get_size().y());
        setArg(map.get_size().z());
        setArg(map.get_pos().x());
        setArg(map.get_pos().y());
        setArg(map.get_pos().z());
        setArg(map.get_offset().x());
        setArg(map.get_offset().y());
        setArg(map.get_offset().z());
        for (int i = 0; i < SPLIT_FACTOR; i++)
        {
            setArg(new_entries.getBuffer());
        }
        setArg(tau);
        setArg(max_weight);

        //setArg(up);
        setArg(up.x);
        setArg(up.y);
        setArg(up.z);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan_points.getBuffer(), new_entries.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels