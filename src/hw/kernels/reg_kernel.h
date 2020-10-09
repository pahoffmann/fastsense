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

    /**
     * @brief interface between the software and the hw, calls the run method of the kernel, writes all the data coming from the kernel 
     *        into the datatypes used by the software
     * 
     * @param map           current local map
     * @param scan_points   points from the current velodyne scan
     * @param local_h       reference for the local h matrix used by the software
     * @param local_g       reference for the local g matrix used by the software
     * @param local_error   local error ref
     * @param local_count   local count ref
     */
    void synchronized_run(map::LocalMap& map, ScanPoints_t& scan_points, Eigen::Matrix<long, 6, 6> &local_h, Eigen::Matrix<long, 6, 1> &local_g, int &local_error, int &local_count)
    {
        auto queue = fastsense::hw::FPGAManager::create_command_queue();
         buffer::OutputBuffer<long> outbuf(queue, 44); //matrix buffer for g matrix TODO: determine if this needs to be in registration.cpp

        run(map, scan_points, outbuf, queue);
        
        waitComplete();

        //std::cout << "Kernel_H " << __LINE__ << std::endl;

        local_h << outbuf[0],  outbuf[1], outbuf[2], outbuf[3], outbuf[4], outbuf[5],
                   outbuf[6], outbuf[7], outbuf[8], outbuf[9], outbuf[10], outbuf[11],
                   outbuf[12], outbuf[13], outbuf[14], outbuf[15], outbuf[16], outbuf[17],
                   outbuf[18], outbuf[19], outbuf[20], outbuf[21], outbuf[22], outbuf[23],
                   outbuf[24], outbuf[25], outbuf[26], outbuf[27], outbuf[28], outbuf[29],
                   outbuf[30], outbuf[31], outbuf[32], outbuf[33], outbuf[34], outbuf[35];

        local_g << outbuf[36],
                   outbuf[37],
                   outbuf[38],
                   outbuf[39],
                   outbuf[40],
                   outbuf[41];

        local_error = outbuf[42];
        local_count = outbuf[43];
    }

    /**
     * @brief Called by the synchronized run method, uses the kernel to register
     * 
     * @param map 
     * @param scan_points 
     * @param outbuf 
     * @param queue 
     */
    void run(map::LocalMap& map, ScanPoints_t& scan_points, buffer::OutputBuffer<long> &outbuf, fastsense::CommandQueuePtr queue)
    {
        //std::cout << "Kernel_H " << __LINE__ << std::endl;
        resetNArg();

        buffer::InputOutputBuffer<Point> point_data(queue, scan_points.size());

        for(size_t i = 0; i < scan_points.size(); ++i)
        {
            point_data[i].x = scan_points[i].x();
            point_data[i].y = scan_points[i].y();
            point_data[i].z = scan_points[i].z();
        }

        //std::cout << "Kernel_H " << __LINE__ << std::endl;
        //std::cout << "Point data: size: " << static_cast<int>(point_data.size()) << std::endl << std::endl;

        auto m = map.get_hardware_representation();
        
        setArg(point_data.getBuffer());
        setArg(static_cast<int>(point_data.size()));
        setArg(map.getBuffer().getBuffer());
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
        setArg(outbuf.getBuffer());

        //std::cout << "Kernel_H " << __LINE__ << std::endl;

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        //std::cout << "Kernel_H " << __LINE__ << std::endl;

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        //std::cout << "Kernel_H " << __LINE__ << std::endl;

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
        
        //std::cout << "Kernel_H " << __LINE__ << std::endl;
    }
};

} // namespace fastsense::kernels