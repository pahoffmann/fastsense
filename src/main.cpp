/**
 * @file main.cpp
 * @author Julian Gaal
 * @date 2020-08-09
 */

#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "vadd.h"
#include <hw/buffer/buffer.h>
#include <hw/types.h>

namespace fs = fastsense;

static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

int main(int argc, char* argv[])
{

    //TARGET_DEVICE macro needs to be passed from gcc command line
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <xclbin>" << std::endl;
        return EXIT_FAILURE;
    }

    char* xclbinFilename = argv[1];

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
        std::cout << "Error: Unable to find Target Device "
                  << device.getInfo<CL_DEVICE_NAME>() << std::endl;
        return EXIT_FAILURE;
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

    // This call will get the kernel object from program. A kernel is an
    // OpenCL function that is executed on the FPGA.
    std::cout << "Getting kernel object from program\n";
    cl::Kernel krnl_main(program, "krnl_main");

    // These commands will allocate memory on the Device. The cl::Buffer objects can
    // be used to reference the memory locations on the device.
    std::cout << "Setting up buffers on device\n";
    fs::buffer::InputBuffer<int> buffer_a(q, context, DATA_SIZE);
    fs::buffer::InputBuffer<int> buffer_b(q, context, DATA_SIZE);
    fs::buffer::OutputBuffer<int> buffer_result(q, context, DATA_SIZE);

    //set the kernel Arguments
    int narg = 0;
    std::cout << "Setting kernel arguments\n";
    krnl_main.setArg(narg++, buffer_a.getBuffer());
    krnl_main.setArg(narg++, buffer_b.getBuffer());
    krnl_main.setArg(narg++, buffer_result.getBuffer());
    krnl_main.setArg(narg++, DATA_SIZE);

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

    // Data will be migrated to kernel space
    std::cout << "data to kernel\n";
    q->enqueueMigrateMemObjects({buffer_a.getBuffer(), buffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE/* 0 means from host*/);

    //Launch the Kernel
    std::cout << "launch kernel\n";
    q->enqueueTask(krnl_main);

    // The result of the previous kernel execution will need to be retrieved in
    // order to view the results. This call will transfer the data from FPGA to
    // source_results vector
    q->enqueueMigrateMemObjects({buffer_result.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST);
    std::cout << "load results\n";

    q->finish();

    //Verify the result
    int match = 0;
    for (int i = 0; i < DATA_SIZE; i++)
    {
        int host_result = ptr_a[i] + ptr_b[i];
        if (ptr_result[i] != host_result)
        {
            printf(error_message.c_str(), i, host_result, ptr_result[i]);
            match = 1;
            break;
        }
    }

    std::cout << "TEST " << (match ? "FAILED" : "PASSED") << std::endl;
    return (match ? EXIT_FAILURE :  EXIT_SUCCESS);
}
