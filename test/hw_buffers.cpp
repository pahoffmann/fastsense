/**
 * @file hw_buffers.cpp
 * @author Julian Gaal
 */

#include <hw/fpga_manager.h>
#include <iostream>

#include "catch2_config.h"

#include "kernels/io_buffer_test_kernel.h"

using namespace fastsense;

TEST_CASE("Testing IOBuffer", "[TestIOBuffer]")
{
    auto q = hw::FPGAManager::create_command_queue();
    kernels::IOBufferTestKernel kernel{q};

    constexpr int DATA_SIZE = 100;
    buffer::InputOutputBuffer<int> data;
    std::fill_n(data.begin(), DATA_SIZE, 1);

    kernel.run(data, DATA_SIZE);
    kernel.waitComplete();

    SECTION("Test 'InputOutputBuffer'")
    {
        std::cout << "    Section 'InputOutputBuffer'\n";
        std::for_each(data.begin(), data.end(), [](int d) { REQUIRE(d == 2); });
    }
}