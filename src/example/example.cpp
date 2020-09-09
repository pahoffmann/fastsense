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

#include <catch2/catch.hpp>

namespace fs = fastsense;

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test krnl_vadd.cpp", "")
{
    const char* xclbinFilename = "FastSense.exe.xclbin";

    // Creates a vector of DATA_SIZE elements with an initial value of 10 and 32
    // using customized allocator for getting buffer alignment to 4k boundary
    std::vector<cl::Device> devices;
    cl::Device device;
    std::vector<cl::Platform> platforms;
    bool found_device = false;

    //traversing all Platforms To find Xilinx Platform and targeted
    //Device in Xilinx Platform
    cl::Platform::get(&platforms);
    for (size_t i = 0; (i < platforms.size() ) & (found_device == false) ; i++)
    {
        cl::Platform platform = platforms[i];
        std::string platformName = platform.getInfo<CL_PLATFORM_NAME>();
        if ( platformName == "Xilinx")
        {
            devices.clear();
            platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices);
            if (devices.size())
            {
                device = devices[0];
                found_device = true;
                break;
            }
        }
    }
    if (found_device == false)
    {
        throw std::runtime_error(("Error: Unable to find Target Device " + device.getInfo<CL_DEVICE_NAME>()));
    }

    // Creating Context and Command Queue for selected device
    cl::Context context(device);
    fs::CommandQueuePtr q = std::make_shared<cl::CommandQueue>(context, device, CL_QUEUE_PROFILING_ENABLE);

    // Load xclbin
    std::cout << "Loading: '" << xclbinFilename << "'\n";
    std::ifstream bin_file(xclbinFilename, std::ifstream::binary);
    bin_file.seekg (0, bin_file.end);
    unsigned nb = bin_file.tellg();
    bin_file.seekg (0, bin_file.beg);
    char* buf = new char [nb];
    bin_file.read(buf, nb);

    // Creating Program from Binary File
    std::cout << "Creating Program from Binary File\n";
    cl::Program::Binaries bins;
    bins.push_back({buf, nb});
    devices.resize(1);
    cl::Program program(context, devices, bins);

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.
    std::cout << "Setting up buffers on device\n";
    fs::buffer::InputBuffer<int> buffer_a(q, context, DATA_SIZE);
    fs::buffer::InputBuffer<int> buffer_b(q, context, DATA_SIZE);
    fs::buffer::OutputBuffer<int> buffer_result(q, context, DATA_SIZE);

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

    fs::kernels::VaddKernel krnl_vadd(q, program);
    krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);

    //Verify the result
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int host_result = ptr_a[i] + ptr_b[i];
        REQUIRE(ptr_result[i] == host_result);
    }
}
