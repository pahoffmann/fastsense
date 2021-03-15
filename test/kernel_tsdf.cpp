/**
 * @author Marc Eisoldt
 *
 * Test the hardware implementation of the TSDF generatio and map update with simple scenarios
 */

#include <tsdf/krnl_tsdf.h>

#include "catch2_config.h"

using namespace fastsense;

constexpr int RINGS = 16;
constexpr float VERTICAL_FOV_ANGLE = 30;

TEST_CASE("Kernel_TSDF", "[kernel]")
{
    std::cout << "Testing 'Kernel_TSDF'" << std::endl;

    CommandQueuePtr q = hw::FPGAManager::create_command_queue();

    constexpr int SCALE = MAP_RESOLUTION;

    constexpr int TAU = 3 * SCALE;

    constexpr int SIZE_X = 50 * SCALE / MAP_RESOLUTION;
    constexpr int SIZE_Y = 50 * SCALE / MAP_RESOLUTION;
    constexpr int SIZE_Z = 10 * SCALE / MAP_RESOLUTION;

    SECTION("Generation")
    {
        std::cout << "    Section 'Generation'" << std::endl;

        auto gm_ptr = std::make_shared<map::GlobalMap>("MapTest.h5", 0, 0);
        map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

        buffer::InputBuffer<PointHW> kernel_points(q, 1);
        kernel_points[0].x = 6 * SCALE + MAP_RESOLUTION / 2;
        kernel_points[0].y = MAP_RESOLUTION / 2;
        kernel_points[0].z = MAP_RESOLUTION / 2;

        tsdf::TSDFKernel krnl(q, localMap.getBuffer().size());
        krnl.synchronized_run(localMap, kernel_points, kernel_points.size(), TAU, 5, RINGS, VERTICAL_FOV_ANGLE);

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
        std::cout << "    Section 'Update'" << std::endl;

        constexpr int DEFAULT_WEIGHT = 8;

        auto gm_ptr = std::make_shared<map::GlobalMap>("MapTest.h5", 0, DEFAULT_WEIGHT);
        map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

        buffer::InputBuffer<PointHW> kernel_points(q, 1);
        kernel_points[0].x = 6 * SCALE + MAP_RESOLUTION / 2;
        kernel_points[0].y = MAP_RESOLUTION / 2;
        kernel_points[0].z = MAP_RESOLUTION / 2;

        tsdf::TSDFKernel krnl(q, localMap.getBuffer().size());
        krnl.synchronized_run(localMap, kernel_points, kernel_points.size(), TAU, 5, RINGS, VERTICAL_FOV_ANGLE);

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

        krnl.synchronized_run(localMap, kernel_points, kernel_points.size(), TAU, 1, RINGS, VERTICAL_FOV_ANGLE);
        krnl.waitComplete();

        CHECK(localMap.value(6, 0, 0).weight() == WEIGHT_RESOLUTION);
    }
}
