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
    std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, 0);
    auto commandQueue = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

    Vector3i scanner_pos(0, 0, 0);

    ScanPoints_t points;
    //points.push_back(Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2));
    points.push_back(Vector3i(6, 0, 0));

    auto q = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::kernels::TSDFKernel krnl(q);

    krnl.run(localMap, points);
    krnl.waitComplete();

    auto entry = localMap.value(6, 0, 0);

    std::cout << entry.first << " " << entry.second << std::endl;;

    SECTION("TSDF Kernel")
    {
    }
}