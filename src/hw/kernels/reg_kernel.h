#pragma once

/**
 * @author Patrick Hoffmann
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <iostream>
#include <util/types.h>
#include <eigen3/Eigen/Dense>

struct Point
{
    int x;
    int y;
    int z;
};


namespace fastsense::kernels
{

class RegistrationKernel : public BaseKernel
{
public:
    RegistrationKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_reg"}
    {}

    ~RegistrationKernel() = default;

    void run(map::LocalMap& map, ScanPoints_t& scan_points, Eigen::Matrix<long, 6, 6> &local_h, Eigen::Matrix<long, 6, 1> &local_g, int &local_error, int &local_count)
    {
        resetNArg();
        setArg(map.getBuffer().getBuffer());

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        buffer::InputOutputBuffer<Point> point_data(queue, scan_points.size());

        for(size_t i = 0; i < scan_points.size(); ++i)
        {
            point_data[i].x = scan_points[i].x();
            point_data[i].y = scan_points[i].y();
            point_data[i].z = scan_points[i].z();
        }

        buffer::OutputBuffer<long> outbuf(queue, 44); //matrix buffer for g matrix TODO: determine if this needs to be in registration.cpp

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
        setArg(MAP_RESOLUTION);
        setArg(point_data);
        setArg(scan_points.size());
        setArg(outbuf.getBuffer());

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
        
        //TODO: does this even work? - like this, sw and hw are completely encapsulated
        waitComplete();

        //TODO: better way to do this? does it work?

        for(int h = 0; h < 36; h++)
        {
            local_h << outbuf[h];
        }

        for(int g = 36; g < 42; g++)
        {
            local_g << outbuf[g];
        }

        local_error = outbuf[42];
        local_count = outbuf[43];
    }
};

} // namespace fastsense::kernels