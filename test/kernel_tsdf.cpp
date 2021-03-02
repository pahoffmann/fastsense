/**
 * @author Marc Eisoldt
 *
 * Test the hardware implementation of the TSDF generatio and map update
 * with simple scenarios and a real point cloud
 */


#include <stdlib.h>

#include <hw/kernels/vadd_kernel.h>
#include <registration/registration.h>
#include <util/pcd/pcd_file.h>
#include <tsdf/krnl_tsdf_sw.h>
#include <tsdf/update_tsdf.h>

#include "catch2_config.h"

using fastsense::util::PCDFile;

namespace fastsense::tsdf
{

static void check_tsdf(const fastsense::buffer::InputOutputBuffer<TSDFValue>& tsdf_hw, const fastsense::buffer::InputOutputBuffer<TSDFValue>& tsdf_sw)
{
    REQUIRE(tsdf_hw.size() == tsdf_sw.size());

    size_t err_count = 0;

    for (size_t i = 0; i < tsdf_hw.size(); ++i)
    {
        if (tsdf_hw[i].value() != tsdf_sw[i].value())
        {
            ++err_count;
        }
    }

    REQUIRE(err_count == 0);
}

TEST_CASE("Kernel_TSDF", "[kernel][slow]")
{
    std::cout << "Testing 'Kernel_TSDF'" << std::endl;

    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    SECTION("Simple_Scenario")
    {
        std::cout << "    Section 'Simple_Scenario'" << std::endl;

        constexpr int SCALE = MAP_RESOLUTION;

        constexpr int TAU = 3 * SCALE;

        constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

        SECTION("Generation")
        {
            std::cout << "        Section 'Generation'" << std::endl;

            std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, 0);
            fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

            fastsense::kernels::TSDFKernel krnl(q, localMap.getBuffer().size());


            Vector3i scanner_pos(0, 0, 0);

            fastsense::buffer::InputBuffer<PointHW> kernel_points(q, 1);

            kernel_points[0].x = 6 * SCALE + MAP_RESOLUTION / 2;
            kernel_points[0].y = MAP_RESOLUTION / 2;
            kernel_points[0].z = MAP_RESOLUTION / 2;

            krnl.run(localMap, kernel_points, kernel_points.size(), TAU, 5 * WEIGHT_RESOLUTION);
            krnl.waitComplete();

            // Front values
            CHECK(localMap.value(6, 0, 0).value() == 0);
            CHECK(localMap.value(5, 0, 0).value() == 1 * SCALE);
            CHECK(localMap.value(4, 0, 0).value() == 2 * SCALE);
            CHECK(localMap.value(3, 0, 0).value() == TAU);
            CHECK(localMap.value(2, 0, 0).value() == TAU);
            CHECK(localMap.value(1, 0, 0).value() == TAU);

            // Front weights
            CHECK(localMap.value(6, 0, 0).weight() == WEIGHT_RESOLUTION);
            CHECK(localMap.value(5, 0, 0).weight() == WEIGHT_RESOLUTION);
            CHECK(localMap.value(4, 0, 0).weight() == WEIGHT_RESOLUTION);
            CHECK(localMap.value(3, 0, 0).weight() == WEIGHT_RESOLUTION);
            CHECK(localMap.value(2, 0, 0).weight() == WEIGHT_RESOLUTION);
            CHECK(localMap.value(1, 0, 0).weight() == WEIGHT_RESOLUTION);

            // back values
            CHECK(localMap.value( 7, 0, 0).value() == -1 * SCALE);
            CHECK(localMap.value( 8, 0, 0).value() == -2 * SCALE);
            CHECK(localMap.value( 9, 0, 0).value() ==  0 * SCALE);
            CHECK(localMap.value(10, 0, 0).value() ==  0 * SCALE);
            CHECK(localMap.value(11, 0, 0).value() ==  0 * SCALE);
            CHECK(localMap.value(12, 0, 0).value() ==  0 * SCALE);

            // back weights
            CHECK(localMap.value( 7, 0, 0).weight() < WEIGHT_RESOLUTION);
            CHECK(localMap.value( 8, 0, 0).weight() < WEIGHT_RESOLUTION);
            CHECK(localMap.value( 9, 0, 0).weight() == 0);
            CHECK(localMap.value(10, 0, 0).weight() == 0);
            CHECK(localMap.value(11, 0, 0).weight() == 0);
            CHECK(localMap.value(12, 0, 0).weight() == 0);
        }

        SECTION("Update")
        {
            std::cout << "        Section 'Update'" << std::endl;

            constexpr int DEFAULT_WEIGHT = 8;

            std::shared_ptr<fastsense::map::GlobalMap> gm_ptr = std::make_shared<fastsense::map::GlobalMap>("MapTest.h5", 0, DEFAULT_WEIGHT);
            fastsense::map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

            fastsense::kernels::TSDFKernel krnl(q, localMap.getBuffer().size());


            Vector3i scanner_pos(0, 0, 0);

            fastsense::buffer::InputBuffer<PointHW> kernel_points(q, 1);

            kernel_points[0].x = 6 * SCALE + MAP_RESOLUTION / 2;
            kernel_points[0].y = MAP_RESOLUTION / 2;
            kernel_points[0].z = MAP_RESOLUTION / 2;

            krnl.run(localMap, kernel_points, kernel_points.size(), TAU, 5 * WEIGHT_RESOLUTION);
            krnl.waitComplete();

            int new_weight = WEIGHT_RESOLUTION + DEFAULT_WEIGHT;

            // Front values
            CHECK(localMap.value(6, 0, 0).value() == 0);
            CHECK(localMap.value(5, 0, 0).value() == 1 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
            CHECK(localMap.value(4, 0, 0).value() == 2 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
            CHECK(localMap.value(3, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
            CHECK(localMap.value(2, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
            CHECK(localMap.value(1, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);

            // Front weights
            CHECK(localMap.value(6, 0, 0).weight() == new_weight);
            CHECK(localMap.value(5, 0, 0).weight() == new_weight);
            CHECK(localMap.value(4, 0, 0).weight() == new_weight);
            CHECK(localMap.value(3, 0, 0).weight() == new_weight);
            CHECK(localMap.value(2, 0, 0).weight() == new_weight);
            CHECK(localMap.value(1, 0, 0).weight() == new_weight);

            krnl.run(localMap, kernel_points, kernel_points.size(), TAU, 24);
            krnl.waitComplete();

            CHECK(localMap.value(6, 0, 0).weight() == 24);
        }
    }

    SECTION("Simulation_Cloud")
    {
        std::cout << "    Section 'Simple_Cloud'" << std::endl;

        constexpr unsigned int SCALE = 1000;
        constexpr float TAU = 1 * SCALE;
        constexpr float MAX_WEIGHT = WEIGHT_RESOLUTION - 1;

        constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
        constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

        std::vector<std::vector<Vector3f>> float_points;
        unsigned int num_points;

        fastsense::util::PCDFile file("sim_cloud.pcd");
        file.readPoints(float_points, num_points);

        auto count = 0u;

        fastsense::buffer::InputBuffer<PointHW> kernel_points(q, num_points);

        std::vector<PointHW> kernel_points_sw(num_points);

        for (const auto& ring : float_points)
        {
            for (const auto& point : ring)
            {
                kernel_points[count].x = point.x() * SCALE;;
                kernel_points[count].y = point.y() * SCALE;;
                kernel_points[count].z = point.z() * SCALE;;

                kernel_points_sw[count].x = kernel_points[count].x;
                kernel_points_sw[count].y = kernel_points[count].y;
                kernel_points_sw[count].z = kernel_points[count].z;

                ++count;
            }
        }

        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map.h5", 0.0, 0.0));
        fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q);

        fastsense::kernels::TSDFKernel krnl(q, local_map.getBuffer().size());

        std::shared_ptr<fastsense::map::GlobalMap> global_map_sw_ptr(new fastsense::map::GlobalMap("test_global_map_sw.h5", 0.0, 0.0));
        fastsense::map::LocalMap local_map_sw(SIZE_X, SIZE_Y, SIZE_Z, global_map_sw_ptr, q);

        //calc tsdf values for the points from the pcd and store them in the local map

        krnl.run(local_map, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
        krnl.waitComplete();

        fastsense::buffer::InputOutputBuffer<TSDFValue> new_entries(q, local_map_sw.get_size().x() * local_map_sw.get_size().y() * local_map_sw.get_size().z());

        for (int i = 0; i < local_map.get_size().x() * local_map.get_size().y() * local_map.get_size().z(); ++i)
        {
            new_entries[i].value(0);
            new_entries[i].weight(0);
        }

        auto& size = local_map_sw.get_size();
        auto& pos = local_map_sw.get_pos();
        auto& offset = local_map_sw.get_offset();
        PointHW up = PointHW(0, 0, MATRIX_RESOLUTION);

        fastsense::tsdf::krnl_tsdf_sw(kernel_points_sw.data(),
                                      kernel_points_sw.data(),
                                      kernel_points_sw.data(),
                                      kernel_points_sw.data(),
                                      num_points,
                                      (TSDFValueHW*)local_map_sw.getBuffer().getVirtualAddress(),
                                      (TSDFValueHW*)local_map_sw.getBuffer().getVirtualAddress(),
                                      (TSDFValueHW*)local_map_sw.getBuffer().getVirtualAddress(),
                                      (TSDFValueHW*)local_map_sw.getBuffer().getVirtualAddress(),
                                      size.x(), size.y(), size.z(),
                                      pos.x(), pos.y(), pos.z(),
                                      offset.x(), offset.y(), offset.z(),
                                      (TSDFValueHW*)new_entries.getVirtualAddress(),
                                      (TSDFValueHW*)new_entries.getVirtualAddress(),
                                      (TSDFValueHW*)new_entries.getVirtualAddress(),
                                      (TSDFValueHW*)new_entries.getVirtualAddress(),
                                      TAU,
                                      MAX_WEIGHT,
                                      up.x, up.y, up.z);


        check_tsdf(local_map.getBuffer(), local_map_sw.getBuffer());
    }
}

} //namespace fastsense::tsdf