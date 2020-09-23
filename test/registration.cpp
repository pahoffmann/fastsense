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
#include <util/types.h>

#include "catch2_config.h"


namespace fastsense::registration{


typedef 
static const int DATA_SIZE = 4096;

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Test Registration", "")
{
    // const char* xclbinFilename = "FastSense.xclbin";

    // fs::hw::FPGAManager::loadXCLBIN(xclbinFilename);

    // fs::CommandQueuePtr q = fs::hw::FPGAManager::createCommandQueue();

    // // These commands will allocate memory on the Device. The cl::Buffer objects can
    // // be used to reference the memory locations on the device.
    // fs::buffer::InputBuffer<int> buffer_a{q, DATA_SIZE};
    // fs::buffer::InputBuffer<int> buffer_b{q, DATA_SIZE};
    // fs::buffer::OutputBuffer<int> buffer_result{q, DATA_SIZE};

    // //setting input data
    // for (int i = 0 ; i < DATA_SIZE; i++)
    // {
    //     buffer_a[i] = 10;
    //     buffer_b[i] = 20;
    // }

    // fs::kernels::VaddKernel krnl_vadd{q};

    // krnl_vadd.run(buffer_a, buffer_b, buffer_result, DATA_SIZE);
    // krnl_vadd.waitComplete();

    // //Verify the result
    // for (int i = 0; i < DATA_SIZE; i++)
    // {
    //     int host_result = buffer_a[i] + buffer_b[i];
    //     REQUIRE(buffer_result[i] == host_result);
    // }


    // //create a registration object:
    // Registration registration = new Registration();

    // //use registration object to calculate a specific transformation, for example: create a pointcloud, transform it and calculate the transformation using the registration
    // //might be a basic cloud with random data - or to make it random, but repeatable: use seeds.


    // create pcl from velodyne sample, create local map, transform pcl and see what the reconstruction can do.
    SECTION("Test Translation")
    {
        float tx, ty, tz = 0.0f;
        Mat4 translation_mat;
        translation_mat << 1, 0, 0, tx,
                           0, 1, 0, ty,
                           0, 0, 1, tz,
                           0, 0, 0, 1;
    }
    SECTION("Test Rotation")
    {
        float rx = 20.0 * (M_PI/180); //radiants
        Mat4 translation_mat_y;
        translation_mat_y << cos(rx),     0, sin(rx), 0,
                           0,           1, 0,       0,
                           -1* sin(rx), 0, cos(rx), 0,
                           0,           0, 0,       1;
    }
}

} //namespace fastsense:: registration