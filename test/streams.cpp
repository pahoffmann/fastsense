/**
 * @file hw_buffers.cpp
 * @author Julian Gaal
 */

#include <iostream>
#include "kernels/streamin_kernel.h"
#include "kernels/streamout_kernel.h"
#include "catch2_config.h"

using namespace fastsense;

TEST_CASE("Streams", "[Streams]")
{
    auto q1 = hw::FPGAManager::create_command_queue();
    auto q2 = hw::FPGAManager::create_command_queue();
    kernels::StreamInKernel kernelIn{q1};
    kernels::StreamOutKernel kernelOut{q2};
    std::cout << "Kernels initialized" << std::endl;

    constexpr int DATA_SIZE = 256;
    buffer::InputBuffer<int> dataIn{q2, DATA_SIZE};
    buffer::OutputBuffer<int> dataOut{q1, DATA_SIZE};
    std::cout << "Buffers initialized" << std::endl;
    std::iota(dataIn.begin(), dataIn.end(), 0);
    std::fill(dataOut.begin(), dataOut.end(), -1);
    std::cout << "Buffers filled" << std::endl;

    kernelOut.run(dataIn);
    std::cout << "Kernel out run" << std::endl;
    kernelIn.run(dataOut);
    std::cout << "Kernel in run" << std::endl;
    kernelOut.waitComplete();
    std::cout << "Kernel out complete" << std::endl;
    kernelIn.waitComplete();
    std::cout << "Kernel in complete" << std::endl;

    for(int i = 0; i < DATA_SIZE; i++)
    {
        REQUIRE(dataIn[i] == dataOut[i]);
    }
}