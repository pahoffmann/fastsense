/**
 * @file main.cpp
 * @author Julian Gaal
 * @date 2020-08-09
 */

#define CATCH_CONFIG_MAIN

#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "vadd.h"
#include <hw/buffer/buffer.h>
#include <hw/kernels/vadd_kernel.h>
#include <hw/types.h>
#include <hw/fpga_program.h>

#include <catch2/catch.hpp>

namespace fs = fastsense;

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test krnl_vadd.cpp", "")
{
    const char* xclbinFilename = "FastSense.exe.xclbin";

    fs::hw::FPGAProgram program{xclbinFilename};
    fs::CommandQueuePtr q = std::make_shared<cl::CommandQueue>(program.getContext(), program.getDevice(), CL_QUEUE_PROFILING_ENABLE);

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.
    std::cout << "Setting up buffers on device\n";
    fs::buffer::InputBuffer<int> buffer_a{q, program.getContext(), DATA_SIZE};
    fs::buffer::InputBuffer<int> buffer_b{q, program.getContext(), DATA_SIZE};
    fs::buffer::OutputBuffer<int> buffer_result{q, program.getContext(), DATA_SIZE};

    //We then need to map our OpenCL buffers to get the pointers
    std::cout << "map buffers to virtual addresses\n";
    int* ptr_a = buffer_a.getVirtualAddress();
    int* ptr_b = buffer_b.getVirtualAddress();
    int* ptr_result = buffer_result.getVirtualAddress();

    //setting input data
    for (int i = 0 ; i < DATA_SIZE; i++)
    {
        ptr_a[i] = 10;
        ptr_b[i] = 20;
    }

    fs::kernels::VaddKernel krnl_vadd{q, program};
    krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);

    //Verify the result
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int host_result = ptr_a[i] + ptr_b[i];
        REQUIRE(ptr_result[i] == host_result);
    }
}
