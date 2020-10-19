/**
 * @author Marc Eisoldt
 */
#include "catch2_config.h"
#include <tsdf/ProjectionNormal.h>
#include <tsdf/update_tsdf.h>
#include <hw/buffer/buffer.h>
#include <eval/runtime_evaluator.h>
#include <util/pcd/pcd_file.h>
#include <iostream>

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

        update_tsdf(points, scanner_pos, localMap, TAU, 100);

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
        std::cout << "    Section 'TSDF Update'" << std::endl;
        // TODO

        std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", 0, 7 * WEIGHT_RESOLUTION);
        auto commandQueue = FPGAManager::create_command_queue();
        LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, commandQueue};

        Vector3i scanner_pos(0, 0, 0);
        
        ScanPoints_t points(1);
        points[0] = Vector3i(6, 0, 0) * SCALE + Vector3i::Constant(MAP_RESOLUTION / 2);

        update_tsdf(points, scanner_pos, localMap, TAU, 100 * WEIGHT_RESOLUTION);

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
            update_tsdf(points, scanner_pos, localMap, TAU, WEIGHT_RESOLUTION);
            CHECK(localMap.value(6, 0, 0).second == WEIGHT_RESOLUTION);
        }
    }

    // SECTION("TSDF Runtime")
    // {
    //     constexpr unsigned int SCALE = 100;
    //     constexpr float TAU = 1 * SCALE;
    //     constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

    //     constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
    //     constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
    //     constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION; 

    //     RuntimeEvaluator eval;

    //     fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    //     std::vector<std::vector<Vector3f>> float_points;
    //     unsigned int num_points;

    //     fastsense::util::PCDFile file("sim_cloud.pcd");
    //     file.readPoints(float_points, num_points);

    //     auto count = 0u;

    //     ScanPoints_t scan_points(num_points);

    //     for(const auto& ring : float_points)
    //     {
    //         for(const auto& point : ring)
    //         {
    //             scan_points[count].x() = point.x() * SCALE;
    //             scan_points[count].y() = point.y() * SCALE;
    //             scan_points[count].z() = point.z() * SCALE;
                
    //             ++count;
    //         }
    //     }

    //     std::cout << "num points: " << count << std::endl;

    //     std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
    //     fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q);

    //     // Initialize temporary testing variables

    //     //calc tsdf values for the points from the pcd and store them in the local map

    //     for(int i = 0; i < 100; i++)
    //     {
    //         eval.start("TSDF update");

    //         fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map, TAU, MAX_WEIGHT);

    //         eval.stop();
    //     }

    //     std::cout << eval.to_string() << std::endl;
    // }
    
    SECTION("TSDF Interpolation")
    {
        std::cout << "    Section 'TSDF Interpolation'" << std::endl;
        // TODO
    }

    SECTION("Complete Scan")
    {
        std::cout << "    Section 'Complete Scan'" << std::endl;
        // TODO
    }
}
