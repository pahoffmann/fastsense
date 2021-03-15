#pragma once

/**
 * @file krnl_tsdf.h
 * @author Marc Eisoldt
 * @author Malte Hillmann
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <util/point_hw.h>
#include <hw/buffer/buffer.h>

#include <iostream>

namespace fastsense::tsdf
{

class TSDFKernel : public kernels::BaseKernel
{
    buffer::InputOutputBuffer<TSDFValue> new_entries;

public:

    /**
     * @brief Create a new TSDF kernel
     *
     * @param queue The CommandQueue for kernel operations
     * @param map_size The size of the 1D Array in the LocalMap
     */
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

    void synchronized_run(map::LocalMap& map,
                          const buffer::InputBuffer<PointHW>& scan_points,
                          int num_points,
                          int tau,
                          float max_weight_float,
                          int rings,
                          float vertical_fov_angle,
                          PointHW up = PointHW(0, 0, MATRIX_RESOLUTION))
    {
        int max_weight = max_weight_float * WEIGHT_RESOLUTION;
        int dz_per_distance = std::tan(vertical_fov_angle / (rings - 1.0) / 180.0 * M_PI) / 2.0 * MATRIX_RESOLUTION;

        run(map,
            scan_points,
            num_points,
            tau,
            max_weight,
            dz_per_distance,
            up);

        waitComplete();
    }

    void run(map::LocalMap& map,
             const buffer::InputBuffer<PointHW>& scan_points,
             int num_points,
             TSDFValue::ValueType tau,
             TSDFValue::WeightType max_weight,
             int dz_per_distance,
             PointHW up = PointHW(0, 0, MATRIX_RESOLUTION))
    {
        for (auto& v : new_entries)
        {
            v = TSDFValue(0, 0);
        }

        auto m = map.get_hardware_representation();

        resetNArg();
        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(scan_points.getBuffer());
        }
        setArg(num_points);
        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(map.getBuffer().getBuffer());
        }

        setArgs(m.sizeX,   m.sizeY,   m.sizeZ);
        setArgs(m.posX,    m.posY,    m.posZ);
        setArgs(m.offsetX, m.offsetY, m.offsetZ);

        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(new_entries.getBuffer());
        }
        setArg(tau);
        setArg(max_weight);
        setArg(dz_per_distance);
        setArgs(up.x, up.y, up.z);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan_points.getBuffer(), new_entries.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::tsdf