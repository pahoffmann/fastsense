/**
 * @file main.cpp
 * @author Julian Gaal
 * @date 2020-08-09
 */

#define CATCH_CONFIG_MAIN

#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>
#include <hw/types.h>
#include <hw/fpga_manager.h>

#include <catch2/catch.hpp>

namespace fs = fastsense;

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test krnl_vadd.cpp", "")
{
    const char* xclbinFilename = "FastSense.exe.xclbin";

    fs::hw::FPGAManager manager{xclbinFilename};

    fs::CommandQueuePtr q = manager.createCommandQueue();

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.
    fs::buffer::InputBuffer<int> buffer_a{q, manager.getContext(), DATA_SIZE};
    fs::buffer::InputBuffer<int> buffer_b{q, manager.getContext(), DATA_SIZE};
    fs::buffer::OutputBuffer<int> buffer_result{q, manager.getContext(), DATA_SIZE};

    //setting input data
    for (int i = 0 ; i < DATA_SIZE; i++)
    {
        buffer_a[i] = 10;
        buffer_b[i] = 20;
    }

    fs::kernels::VaddKernel krnl_vadd{q, manager.getProgram()};

    krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
    krnl_vadd.waitComplete();

    //Verify the result
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int host_result = buffer_a[i] + buffer_b[i];
        REQUIRE(buffer_result[i] == host_result);
    }
}
