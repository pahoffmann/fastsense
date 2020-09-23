/**
 * @file main.cpp
 * @author Patrick
 * @date 2020-08-09
 * Test file used to test the registration implementation
 */


#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>
#include <hw/types.h>
#include <hw/fpga_manager.h>
#include <registration/registration.h>

#include "catch2_config.h"

namespace fastsense::registration{

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test krnl_vadd.cpp", "")
{
    const char* xclbinFilename = "FastSense.xclbin";

    //fastsense::hw::FPGAManager::loadXCLBIN(xclbinFilename);

    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::createCommandQueue();

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.
    fastsense::buffer::InputBuffer<int> buffer_a{q, DATA_SIZE};
    fastsense::buffer::InputBuffer<int> buffer_b{q, DATA_SIZE};
    fastsense::buffer::OutputBuffer<int> buffer_result{q, DATA_SIZE};

    //setting input data
    for (int i = 0 ; i < DATA_SIZE; i++)
    {
        buffer_a[i] = 10;
        buffer_b[i] = 20;
    }

    fastsense::kernels::VaddKernel krnl_vadd{q};

    krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
    krnl_vadd.waitComplete();

    //Verify the result
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int host_result = buffer_a[i] + buffer_b[i];
        REQUIRE(buffer_result[i] != host_result);
    }


    //create a registration object:
    //Registration registration = new Registration();

    //use registration object to calculate a specific transformation, for example: create a pointcloud, transform it and calculate the transformation using the registration
    //might be a basic cloud with random data - or to make it random, but repeatable: use seeds.
}

} //namespace fastsense:: registration