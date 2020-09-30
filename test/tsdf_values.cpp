/**
 * @author Marc Eisoldt
 */

#include "catch2_config.h"
#include <tsdf/ProjectionNormal.h>
#include <tsdf/update_tsdf.h>
#include <iostream>

using namespace fastsense::tsdf;
using namespace fastsense::map;
using namespace fastsense::hw;

constexpr int SCALE = MAP_RESOLUTION;

constexpr int TAU = 3 * SCALE;

constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

constexpr int WEIGHT_SCALE = SCALE / 8.0 /*/ ( 8.0 * WEIGHT_RESOLUTION)*/;

TEST_CASE("TSDF_Values", "[tsdf_values]")
{
    SECTION("TSDF Generation")
    {
        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap_t localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);

        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        update_tsdf(points, scanner_pos, localMap, TAU, 100);

        // Front values
        REQUIRE(localMap.value(6, 0, 0).first == 0);
        REQUIRE(localMap.value(5, 0, 0).first == 1 * SCALE);
        REQUIRE(localMap.value(4, 0, 0).first == 2 * SCALE);
        REQUIRE(localMap.value(3, 0, 0).first == TAU);
        REQUIRE(localMap.value(2, 0, 0).first == TAU);
        REQUIRE(localMap.value(1, 0, 0).first == TAU);

        // Front weights
        REQUIRE(localMap.value(6, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(5, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(4, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(3, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(2, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(1, 0, 0).second == 1 * WEIGHT_RESOLUTION);

        // back values
        REQUIRE(localMap.value( 7, 0, 0).first == -1 * SCALE);
        REQUIRE(localMap.value( 8, 0, 0).first == -2 * SCALE);
        REQUIRE(localMap.value( 9, 0, 0).first ==  0 * SCALE);
        REQUIRE(localMap.value(10, 0, 0).first ==  0 * SCALE);
        REQUIRE(localMap.value(11, 0, 0).first ==  0 * SCALE);
        REQUIRE(localMap.value(12, 0, 0).first ==  0 * SCALE);

        // back weights
        REQUIRE((localMap.value( 7, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( 7, 0, 0).second > 0));
        REQUIRE((localMap.value( 8, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( 8, 0, 0).second > 0));
        REQUIRE(localMap.value( 9, 0, 0).second == 0);
        REQUIRE(localMap.value(10, 0, 0).second == 0);
        REQUIRE(localMap.value(11, 0, 0).second == 0);
        REQUIRE(localMap.value(12, 0, 0).second == 0);
    }

    SECTION("TSDF Update")
    {
        // TODO

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 7 * WEIGHT_RESOLUTION);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap_t localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);
        
        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        update_tsdf(points, scanner_pos, localMap, TAU, 100 * WEIGHT_RESOLUTION);

        // Front values
        REQUIRE(localMap.value(6, 0, 0).first == 0);
        REQUIRE(localMap.value(5, 0, 0).first == 1 * WEIGHT_SCALE);
        REQUIRE(localMap.value(4, 0, 0).first == 2 * WEIGHT_SCALE);
        REQUIRE(localMap.value(3, 0, 0).first == 3 * WEIGHT_SCALE);
        REQUIRE(localMap.value(2, 0, 0).first == 3 * WEIGHT_SCALE);
        REQUIRE(localMap.value(1, 0, 0).first == 3 * WEIGHT_SCALE);

        // Front weights
        REQUIRE(localMap.value(6, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(5, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(4, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(3, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(2, 0, 0).second == 8 * WEIGHT_RESOLUTION);
        REQUIRE(localMap.value(1, 0, 0).second == 8 * WEIGHT_RESOLUTION);

        SECTION("TSDF Max Weight")
        {
            update_tsdf(points, scanner_pos, localMap, TAU, WEIGHT_RESOLUTION);
            REQUIRE(localMap.value(6, 0, 0).second == WEIGHT_RESOLUTION);
        }
    }

    SECTION("TSDF Interpolation")
    {
        // TODO
    }

    SECTION("Complete Scan")
    {
        // TODO
    }
}
