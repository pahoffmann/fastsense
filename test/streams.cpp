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
    auto q = hw::FPGAManager::create_command_queue();
    kernels::StreamInKernel kernelIn{q};
    kernels::StreamOutKernel kernelOut{q};

    constexpr int DATA_SIZE = 256;
    buffer::InputBuffer<int> dataIn{q, DATA_SIZE};
    buffer::OutputBuffer<int> dataOut{q, DATA_SIZE};
    std::iota(dataIn.begin(), dataIn.end(), 0);
    std::fill(dataOut.begin(), dataOut.end(), -1);

    kernelOut.run(dataIn);
    kernelIn.run(dataOut);
    kernelOut.waitComplete();
    kernelIn.waitComplete();

    for(int i = 0; i < DATA_SIZE; i++)
    {
        REQUIRE(dataIn[i] == dataOut[i]);
    }
}