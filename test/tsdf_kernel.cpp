#include <map/local_map.h>
#include <hw/fpga_manager.h>
#include <hw/kernels/tsdf_kernel.h>
#include "catch2_config.h"

#include <iostream>

constexpr int SCALE = MAP_RESOLUTION;

constexpr int TAU = 3 * SCALE;

constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

constexpr int WEIGHT_SCALE = SCALE / 8.0;

TEST_CASE("TSDF_Kernel", "[tsdf_kernel]")
{
    // std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, 0);
    // auto commandQueue = fastsense::hw::FPGAManager::create_command_queue();
    // fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

    // Vector3i scanner_pos(0, 0, 0);

    // ScanPoints_t points;
    // //points.push_back(Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2));
    // points.push_back(Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2));

    // auto q = fastsense::hw::FPGAManager::create_command_queue();
    // fastsense::kernels::TSDFKernel krnl(q);

    // krnl.run(localMap, points, scanner_pos, TAU, 100);
    // krnl.waitComplete();

    // auto entry = localMap.value(6, 0, 0);

    // std::cout << entry.first << " " << entry.second << std::endl;;

    // SECTION("TSDF Kernel")
    // {
    // }

    SECTION("TSDF Generation")
    {
        std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);

        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        auto q = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::kernels::TSDFKernel krnl(q);

        krnl.run(localMap, points, scanner_pos, TAU, 100 * WEIGHT_RESOLUTION);
        krnl.waitComplete();

        // Front values
        CHECK(localMap.value(6, 0, 0).first == 0);
        CHECK(localMap.value(5, 0, 0).first == 1 * SCALE);
        CHECK(localMap.value(4, 0, 0).first == 2 * SCALE);
        CHECK(localMap.value(3, 0, 0).first == TAU);
        CHECK(localMap.value(2, 0, 0).first == TAU);
        CHECK(localMap.value(1, 0, 0).first == TAU);

        // Front weights
        CHECK(localMap.value(6, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(5, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(4, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(3, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(2, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(1, 0, 0).second == 1 * WEIGHT_RESOLUTION);

        // back values
        CHECK(localMap.value( 7, 0, 0).first == -1 * SCALE);
        CHECK(localMap.value( 8, 0, 0).first == -2 * SCALE);
        CHECK(localMap.value( 9, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(10, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(11, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(12, 0, 0).first ==  0 * SCALE);

        // back weights
        CHECK((localMap.value( 7, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( 7, 0, 0).second > 0));
        CHECK((localMap.value( 8, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( 8, 0, 0).second > 0));
        CHECK(localMap.value( 9, 0, 0).second == 0);
        CHECK(localMap.value(10, 0, 0).second == 0);
        CHECK(localMap.value(11, 0, 0).second == 0);
        CHECK(localMap.value(12, 0, 0).second == 0);
    }

    SECTION("TSDF Update")
    {
        // TODO

        std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, 7 * WEIGHT_RESOLUTION);
        auto commandQueue = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);
        
        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        auto q = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::kernels::TSDFKernel krnl(q);

        krnl.run(localMap, points, scanner_pos, TAU, 100 * WEIGHT_RESOLUTION);
        krnl.waitComplete();

        // Front values
        CHECK(localMap.value(6, 0, 0).first == 0);
        CHECK(localMap.value(5, 0, 0).first == 1 * WEIGHT_SCALE);
        CHECK(localMap.value(4, 0, 0).first == 2 * WEIGHT_SCALE);
        CHECK(localMap.value(3, 0, 0).first == 3 * WEIGHT_SCALE);
        CHECK(localMap.value(2, 0, 0).first == 3 * WEIGHT_SCALE);
        CHECK(localMap.value(1, 0, 0).first == 3 * WEIGHT_SCALE);

        // Front weights
        CHECK(localMap.value(6, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(5, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(4, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(3, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(2, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(1, 0, 0).second == 8 * WEIGHT_RESOLUTION);

        SECTION("TSDF Max Weight")
        {
            krnl.run(localMap, points, scanner_pos, TAU, WEIGHT_RESOLUTION);
            krnl.waitComplete();

            CHECK(localMap.value(6, 0, 0).second == WEIGHT_RESOLUTION);
        }
    }
}