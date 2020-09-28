/**
 * @file xyz_buffer.cpp
 * @author Julian Gaal
 * @date 2020-09-28
 */

#include "catch2_config.h"
#include "Application.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <msg/imu_msg.h>

using namespace fastsense::util;

TEST_CASE("Test xyz_buffer.h", "[]")
{   
    SECTION("Test Constructors")
    {
        {
            XYZBuffer<int> buffer{1, 2, 3};
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 2);
            REQUIRE(buffer.z() == 3);
        }
        {
            XYZBuffer<int> buffer{6};
            REQUIRE(buffer.x() == 6);
            REQUIRE(buffer.y() == 6);
            REQUIRE(buffer.z() == 6);
        }
    }

    SECTION("Test Operators")
    {
        XYZBuffer<int> buffer{0, 1, 2};
        buffer += 1;
        REQUIRE(buffer.x() == 1);
        REQUIRE(buffer.y() == 2);
        REQUIRE(buffer.z() == 3);

        buffer += buffer;
        REQUIRE(buffer.x() == 2);
        REQUIRE(buffer.y() == 4);
        REQUIRE(buffer.z() == 6);

    }
}