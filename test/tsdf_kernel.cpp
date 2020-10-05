#include <map/local_map.h>
#include <hw/fpga_manager.h>
#include <hw/kernels/tsdf_kernel.h>
#include <eval/RuntimeEvaluator.h>
#include <util/pcd/PCDFile.h>
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

    SECTION("TSDF Runtime")
    {
        constexpr float TAU = 1 * SCALE;
        constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

        constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION; 

        RuntimeEvaluator eval;

        fastsense::CommandQueuePtr map_queue = fastsense::hw::FPGAManager::create_command_queue();

        std::vector<std::vector<Vector3f>> float_points;
        unsigned int num_points;

        fastsense::util::PCDFile file("sim_cloud.pcd");
        file.readPoints(float_points, num_points);

        auto count = 0u;

        ScanPoints_t scan_points(num_points);

        for(const auto& ring : float_points)
        {
            for(const auto& point : ring)
            {
                scan_points[count].x() = point.x() * SCALE;
                scan_points[count].y() = point.y() * SCALE;
                scan_points[count].z() = point.z() * SCALE;
                
                ++count;
            }
        }

        ScanPoints_t scan_points_2(scan_points);
        ScanPoints_t points_pretransformed_trans(scan_points);
        ScanPoints_t points_pretransformed_rot(scan_points);

        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
        fastsense::map::LocalMap local_map(SIZE_Y, SIZE_Y, SIZE_Z, global_map_ptr, map_queue);

        // Initialize temporary testing variables

        //calc tsdf values for the points from the pcd and store them in the local map

        auto q = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::kernels::TSDFKernel krnl(q);

        for(int i = 0; i < 100; i++)
        {
            eval.start("TSDF kernel");

            //fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map, TAU, MAX_WEIGHT);

            krnl.run(local_map, scan_points, Vector3i::Zero(), TAU, MAX_WEIGHT);
            krnl.waitComplete();

            eval.stop();
        }

        std::cout << eval.to_string() << std::endl;
    }
}