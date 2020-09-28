/**
 * @file tsdf_values.cpp
 * @author Marc Eisoldt 
 * @version 0.1
 * @date 2020-09-16
 * 
 * @copyright Copyright (c) 2020
 * 
 */
//#define CATCH_CONFIG_MAIN
#include "catch2_config.h"
#include <tsdf/ProjectionNormal.h>
#include <tsdf/update_tsdf.h>
#include <iostream>

using namespace fastsense::tsdf;
using namespace fastsense::map;
using namespace fastsense::hw;
//using namespace fastsense::kernels;

TEST_CASE("Test TSDF Values", "[]")
{
    /*SECTION("TSDF Generation")
    {
        // TODO

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 0);
        auto commandQueue = FPGAManager::createCommandQueue();
        LocalMap<std::pair<float, float>> localMap{20, 20, 20, gm_ptr, commandQueue};

        int size[3];
        localMap.getSize(size);

        //std::cout << "tsdf map size (simple cases): " << size[0] << " " << size[1] << " " << size[2] << std::endl; 

        Vector3 scanner_pos(0, 0, 0);
        float tau = 3.0;
        TsdfCalculation method = TsdfCalculation::PROJECTION;

        ScanPoints_t<Vector3> points(1);
        points[0].push_back(Vector3(6.5, 0.5, 0.5));

        update_tsdf(points, scanner_pos, localMap, method, tau, 100.0);

        //std::cout << "tsdf value: " << localMap.value(6, 0, 0).first << std::endl;
    
        // Front values
        REQUIRE(localMap.value(6, 0, 0).first == 0);
        REQUIRE(localMap.value(5, 0, 0).first == 1);
        REQUIRE(localMap.value(4, 0, 0).first == 2);
        REQUIRE(localMap.value(3, 0, 0).first == 3);
        REQUIRE(localMap.value(2, 0, 0).first == 3);
        REQUIRE(localMap.value(1, 0, 0).first == 3);
    
        // Back values
    }

    SECTION("TSDF update")
    {
        // TODO
    }

    SECTION("Complete scan")
    {
        // TODO
    }*/
}
