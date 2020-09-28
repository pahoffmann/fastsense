/**
 * @file tsdf_values.cpp
 * @author Marc Eisoldt 
 * @version 0.1
 * @date 2020-09-16
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "catch2_config.h"
#include <tsdf/ProjectionNormal.h>
#include <tsdf/update_tsdf.h>
#include <iostream>

using namespace fastsense::tsdf;
using namespace fastsense::map;
using namespace fastsense::hw;

TEST_CASE("Test TSDF Values", "[]")
{
    SECTION("TSDF Generation")
    {
        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = FPGAManager::createCommandQueue();
        LocalMap<std::pair<float, float>> localMap{25, 25, 25, gm_ptr, commandQueue};

        int size[3];
        localMap.getSize(size);

        Vector3 scanner_pos(0, 0, 0);
        float tau = 3.0;
        TsdfCalculation method = TsdfCalculation::PROJECTION;

        ScanPoints_t<Vector3> points(1);
        points[0].push_back(Vector3(6.5, 0.5, 0.5));

        update_tsdf(points, scanner_pos, localMap, method, tau, 100.0);

        // Front values
        REQUIRE(localMap.value(6, 0, 0).first == 0);
        REQUIRE(localMap.value(5, 0, 0).first == 1);
        REQUIRE(localMap.value(4, 0, 0).first == 2);
        REQUIRE(localMap.value(3, 0, 0).first == 3);
        REQUIRE(localMap.value(2, 0, 0).first == 3);
        REQUIRE(localMap.value(1, 0, 0).first == 3);

        // Front weights
        REQUIRE(localMap.value(6, 0, 0).second == 1);
        REQUIRE(localMap.value(5, 0, 0).second == 1);
        REQUIRE(localMap.value(4, 0, 0).second == 1);
        REQUIRE(localMap.value(3, 0, 0).second == 1);
        REQUIRE(localMap.value(2, 0, 0).second == 1);
        REQUIRE(localMap.value(1, 0, 0).second == 1);

        // back values
        REQUIRE(localMap.value( 7, 0, 0).first == -1);
        REQUIRE(localMap.value( 8, 0, 0).first == -2);
        REQUIRE(localMap.value( 9, 0, 0).first ==  0);
        REQUIRE(localMap.value(10, 0, 0).first ==  0);
        REQUIRE(localMap.value(11, 0, 0).first ==  0);
        REQUIRE(localMap.value(12, 0, 0).first ==  0);

        // back weights
        REQUIRE((localMap.value( 7, 0, 0).second < 1 && localMap.value( 7, 0, 0).second > 0));
        REQUIRE((localMap.value( 8, 0, 0).second < 1 && localMap.value( 8, 0, 0).second > 0));
        REQUIRE((localMap.value( 9, 0, 0).second == 0));
        REQUIRE((localMap.value(10, 0, 0).second == 0));
        REQUIRE((localMap.value(11, 0, 0).second == 0));
        REQUIRE((localMap.value(12, 0, 0).second == 0));
    }

    SECTION("TSDF Update")
    {
        // TODO

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 7);
        auto commandQueue = FPGAManager::createCommandQueue();
        LocalMap<std::pair<float, float>> localMap{25, 25, 25, gm_ptr, commandQueue};

        int size[3];
        localMap.getSize(size);

        Vector3 scanner_pos(0, 0, 0);
        float tau = 3.0;
        TsdfCalculation method = TsdfCalculation::PROJECTION;

        ScanPoints_t<Vector3> points(1);
        points[0].push_back(Vector3(6.5, 0.5, 0.5));
        
        update_tsdf(points, scanner_pos, localMap, method, tau, 100.0);

        // Front values
        REQUIRE(localMap.value(6, 0, 0).first == 0);
        REQUIRE(localMap.value(5, 0, 0).first == 1 / 8.0);
        REQUIRE(localMap.value(4, 0, 0).first == 2 / 8.0);
        REQUIRE(localMap.value(3, 0, 0).first == 3 / 8.0);
        REQUIRE(localMap.value(2, 0, 0).first == 3 / 8.0);
        REQUIRE(localMap.value(1, 0, 0).first == 3 / 8.0);

        // Front weights
        REQUIRE(localMap.value(6, 0, 0).second == 8);
        REQUIRE(localMap.value(5, 0, 0).second == 8);
        REQUIRE(localMap.value(4, 0, 0).second == 8);
        REQUIRE(localMap.value(3, 0, 0).second == 8);
        REQUIRE(localMap.value(2, 0, 0).second == 8);
        REQUIRE(localMap.value(1, 0, 0).second == 8);
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
