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

constexpr auto p1 = 6;
constexpr auto p2 = 5;
constexpr auto p3 = 4;
constexpr auto p4 = 3;
constexpr auto p5 = 2;
constexpr auto p6 = 1;

constexpr auto  p7 =  7;
constexpr auto  p8 =  8;
constexpr auto  p9 =  9;
constexpr auto p10 = 10;
constexpr auto p11 = 11;
constexpr auto p12 = 12;

TEST_CASE("TSDF_Values", "[tsdf_values]")
{
    SECTION("TSDF Generation")
    {
        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap_t localMap{25 * SCALE / MAP_RESOLUTION, 25 * SCALE / MAP_RESOLUTION, 5 * SCALE / MAP_RESOLUTION, gm_ptr, commandQueue};

        int size[3];
        localMap.getSize(size);

        Vector3i scanner_pos = Vector3i::Constant(MAP_RESOLUTION / 2);
        int tau = 3 * SCALE;

        ScanPoints_t points(1);
        points.push_back(Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2));

        update_tsdf(points, scanner_pos, localMap, tau, 100);

        // Front values
        CHECK(localMap.value(p1, 0, 0).first == 0);
        CHECK(localMap.value(p2, 0, 0).first == 1 * SCALE);
        CHECK(localMap.value(p3, 0, 0).first == 2 * SCALE);
        CHECK(localMap.value(p4, 0, 0).first == 3 * SCALE);
        CHECK(localMap.value(p5, 0, 0).first == 3 * SCALE);
        CHECK(localMap.value(p6, 0, 0).first == 3 * SCALE);

        // Front weights
        CHECK(localMap.value(p1, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(p2, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(p3, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(p4, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(p5, 0, 0).second == 1 * WEIGHT_RESOLUTION);
        CHECK(localMap.value(p6, 0, 0).second == 1 * WEIGHT_RESOLUTION);

        // back values
        CHECK(localMap.value( p7, 0, 0).first == -1 * SCALE);
        CHECK(localMap.value( p8, 0, 0).first == -2 * SCALE);
        CHECK(localMap.value( p9, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(p10, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(p11, 0, 0).first ==  0 * SCALE);
        CHECK(localMap.value(p12, 0, 0).first ==  0 * SCALE);

        // back weights
        REQUIRE((localMap.value( p7, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( p7, 0, 0).second > 0));
        REQUIRE((localMap.value( p8, 0, 0).second < 1 * WEIGHT_RESOLUTION && localMap.value( p8, 0, 0).second > 0));
        REQUIRE((localMap.value( p9, 0, 0).second == 0));
        REQUIRE((localMap.value(p10, 0, 0).second == 0));
        REQUIRE((localMap.value(p11, 0, 0).second == 0));
        REQUIRE((localMap.value(p12, 0, 0).second == 0));
    }

    SECTION("TSDF Update")
    {
        // TODO

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 7 * WEIGHT_RESOLUTION);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap_t localMap{25, 25, 25, gm_ptr, commandQueue};

        int size[3];
        localMap.getSize(size);

        Vector3i scanner_pos(0, 0, 0);
        float tau = 3.0;

        ScanPoints_t points(1);
        points.push_back(Vector3i(6.5, 0.5, 0.5));

        update_tsdf(points, scanner_pos, localMap, tau, 100);

        // Front values
        REQUIRE(localMap.value(p1, 0, 0).first == 0);
        REQUIRE(localMap.value(p2, 0, 0).first == 1 / 8.0);
        REQUIRE(localMap.value(p3, 0, 0).first == 2 / 8.0);
        REQUIRE(localMap.value(p4, 0, 0).first == 3 / 8.0);
        REQUIRE(localMap.value(p5, 0, 0).first == 3 / 8.0);
        REQUIRE(localMap.value(p6, 0, 0).first == 3 / 8.0);

        // Front weights
        REQUIRE(localMap.value(p1, 0, 0).second == 8);
        REQUIRE(localMap.value(p2, 0, 0).second == 8);
        REQUIRE(localMap.value(p3, 0, 0).second == 8);
        REQUIRE(localMap.value(p4, 0, 0).second == 8);
        REQUIRE(localMap.value(p5, 0, 0).second == 8);
        REQUIRE(localMap.value(p6, 0, 0).second == 8);
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
