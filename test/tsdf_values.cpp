/**
 * @author Marc Eisoldt
 *
 * Test the software version of the TSDF generation and map update
 */
#include "catch2_config.h"
#include <tsdf/update_tsdf.h>
#include <hw/buffer/buffer.h>
#include <util/pcd/pcd_file.h>
#include <util/constants.h>
#include <iostream>

using namespace fastsense;
using namespace fastsense::tsdf;
using namespace fastsense::map;
using namespace fastsense::hw;

constexpr int SCALE = MAP_RESOLUTION;

constexpr int TAU = 3 * SCALE;

constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

constexpr int WEIGHT_SCALE = SCALE / 8.0;

TEST_CASE("TSDF_Values", "[tsdf_values]")
{
    std::cout << "Testing 'TSDF_Values'" << std::endl;
    SECTION("TSDF Generation")
    {
        std::cout << "    Section 'TSDF Generation'" << std::endl;
        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);

        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        update_tsdf(points, scanner_pos, localMap, TAU, 31);

        // Front values
        CHECK(localMap.value(6, 0, 0).value() == 0);
        CHECK(localMap.value(5, 0, 0).value() == 1 * SCALE);
        CHECK(localMap.value(4, 0, 0).value() == 2 * SCALE);
        CHECK(localMap.value(3, 0, 0).value() == TAU);
        CHECK(localMap.value(2, 0, 0).value() == TAU);
        CHECK(localMap.value(1, 0, 0).value() == TAU);

        // Front weights
        CHECK(localMap.value(6, 0, 0).weight() == 31);
        CHECK(localMap.value(5, 0, 0).weight() == 31);
        CHECK(localMap.value(4, 0, 0).weight() == 31);
        CHECK(localMap.value(3, 0, 0).weight() == 31);
        CHECK(localMap.value(2, 0, 0).weight() == 31);
        CHECK(localMap.value(1, 0, 0).weight() == 31);

        // back values
        CHECK(localMap.value( 7, 0, 0).value() == -1 * SCALE);
        CHECK(localMap.value( 8, 0, 0).value() == -2 * SCALE);
        CHECK(localMap.value( 9, 0, 0).value() ==  0 * SCALE);
        CHECK(localMap.value(10, 0, 0).value() ==  0 * SCALE);
        CHECK(localMap.value(11, 0, 0).value() ==  0 * SCALE);
        CHECK(localMap.value(12, 0, 0).value() ==  0 * SCALE);

        // back weights
        CHECK(localMap.value( 7, 0, 0).weight() < 31);
        CHECK(localMap.value( 8, 0, 0).weight() < 31);
        CHECK(localMap.value( 9, 0, 0).weight() == 0);
        CHECK(localMap.value(10, 0, 0).weight() == 0);
        CHECK(localMap.value(11, 0, 0).weight() == 0);
        CHECK(localMap.value(12, 0, 0).weight() == 0);
    }

    SECTION("TSDF Update")
    {
        std::cout << "    Section 'TSDF Update'" << std::endl;

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 8);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);

        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        update_tsdf(points, scanner_pos, localMap, TAU, 31);

        // Front values
        CHECK(localMap.value(6, 0, 0).value() == 0);
        CHECK(localMap.value(5, 0, 0).value() == 1 * MAP_RESOLUTION * 31 / 39);
        CHECK(localMap.value(4, 0, 0).value() == 2 * MAP_RESOLUTION * 31 / 39);
        CHECK(localMap.value(3, 0, 0).value() == 3 * MAP_RESOLUTION * 31 / 39);
        CHECK(localMap.value(2, 0, 0).value() == 3 * MAP_RESOLUTION * 31 / 39);
        CHECK(localMap.value(1, 0, 0).value() == 3 * MAP_RESOLUTION * 31 / 39);

        // Front weights
        CHECK(localMap.value(6, 0, 0).weight() == 31);
        CHECK(localMap.value(5, 0, 0).weight() == 31);
        CHECK(localMap.value(4, 0, 0).weight() == 31);
        CHECK(localMap.value(3, 0, 0).weight() == 31);
        CHECK(localMap.value(2, 0, 0).weight() == 31);
        CHECK(localMap.value(1, 0, 0).weight() == 31);

        update_tsdf(points, scanner_pos, localMap, TAU, 24);
        CHECK(localMap.value(6, 0, 0).weight() == 24);
    }
}
