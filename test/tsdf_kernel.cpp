#include <map/local_map.h>
#include <hw/fpga_manager.h>
#include <hw/kernels/tsdf_kernel.h>
#include <tsdf/update_tsdf.h>
#include <util/pcd/PCDFile.h>
#include "catch2_config.h"

#include <iostream>

TEST_CASE("TSDF_Kernel", "[tsdf_kernel]")
{
    SECTION("TSDF Runtime")
    {
        constexpr unsigned int SCALE = 100;
        constexpr float TAU = 1 * SCALE;
        constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

        constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION; 

        std::vector<std::vector<Vector3f>> float_points;
        unsigned int num_points;

        fastsense::util::PCDFile file("sim_cloud.pcd");
        file.readPoints(float_points, num_points);

        auto count = 0u;

        ScanPoints_t scan_points(num_points);

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::buffer::InputOutputBuffer<Point> kernel_points(queue, 30000);

        for(const auto& ring : float_points)
        {
            for(const auto& point : ring)
            {
                scan_points[count].x() = point.x() * SCALE;
                scan_points[count].y() = point.y() * SCALE;
                scan_points[count].z() = point.z() * SCALE;

                kernel_points[count].x = scan_points[count].x();
                kernel_points[count].y = scan_points[count].y();
                kernel_points[count].z = scan_points[count].z(); 
                

                ++count;
            }
        }

        std::cout << "num points: " << count << std::endl;

        fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr_compare(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
        fastsense::map::LocalMap local_map_compare(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr_compare, q);

        // Initialize temporary testing variables

        //calc tsdf values for the points from the pcd and store them in the local map

        fastsense::tsdf::update_tsdf(scan_points, Vector3i::Zero(), local_map_compare, TAU, MAX_WEIGHT);

        fastsense::CommandQueuePtr q2 = fastsense::hw::FPGAManager::create_command_queue();    
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map2", 0.0, 0.0));
        fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q2);

        auto q3 = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::kernels::TSDFKernel krnl(q3);

        krnl.run(local_map, kernel_points, Vector3i::Zero(), TAU, MAX_WEIGHT);
        krnl.waitComplete();

        const auto& local_buffer = local_map.getBuffer();
        const auto& compare_buffer = local_map_compare.getBuffer(); 

        int err_count = 0;

        for(int i = 0; i < SIZE_X; ++i)
        {
            for(int j = 0; j < SIZE_Y; ++j)
            {
                for(int k = 0; k < SIZE_Z; ++k)
                {
                    auto index = i + j * SIZE_X + k * SIZE_X * SIZE_Y; 

                    //std::cout << local_buffer[index].first << " " << compare_buffer[index].first << std::endl;

                    REQUIRE(local_buffer[index].first == compare_buffer[index].first);
                }
            }
        }

        //REQUIRE(err_count == 0);
    }
}