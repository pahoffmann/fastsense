/**
 * @file xyz_buffer.cpp
 * @author Julian Gaal
 * @date 2020-09-28
 */

#include "catch2_config.h"
#include <util/config/config_manager.h>
#include <util/logging/logger.h>
#include <msg/imu_msg.h>

using namespace fastsense::util;

TEST_CASE("Test XZYBuffer", "[]")
{
    std::cout << "Testing 'Test XZYBuffer'" << std::endl;   
    SECTION("Test Constructors")
    {
        std::cout << "    Section 'Test Constructors'" << std::endl;
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

    SECTION("Test Operator: +")
    {
        std::cout << "    Section 'Test Operator: +'" << std::endl;   
        {
            auto buffer = XYZBuffer<int>{0, 1, 2} + XYZBuffer<int>{2, 3, 4};
            REQUIRE(buffer.x() == 2);
            REQUIRE(buffer.y() == 4);
            REQUIRE(buffer.z() == 6);
        }

        {
            auto buffer = XYZBuffer<int>{0, 1, 2} + 1;
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 2);
            REQUIRE(buffer.z() == 3);
        }

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

    SECTION("Test Operator: -")
    {
        std::cout << "    Section 'Test Operator: -'" << std::endl;   
        {
            auto buffer = XYZBuffer<int>{0, 1, 2} - XYZBuffer<int>{0, 1, 2};
            REQUIRE(buffer.x() == 0);
            REQUIRE(buffer.y() == 0);
            REQUIRE(buffer.z() == 0);
        }

        {
            auto buffer = XYZBuffer<int>{2, 3, 4} - 1;
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 2);
            REQUIRE(buffer.z() == 3);
        }

        {
            XYZBuffer<int> buffer{2, 3, 4};
            buffer -= 1;
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 2);
            REQUIRE(buffer.z() == 3);

            buffer -= buffer;
            REQUIRE(buffer.x() == 0);
            REQUIRE(buffer.y() == 0);
            REQUIRE(buffer.z() == 0);
        }
    }

    SECTION("Test Operator: *")
    {
        std::cout << "    Section 'Test Operator: *'" << std::endl;   
        {
            auto buffer = XYZBuffer<int>{1, 2, 3} * XYZBuffer<int>{1, 2, 3};
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 4);
            REQUIRE(buffer.z() == 9);
        }

        {
            auto buffer = XYZBuffer<int>{2, 3, 4} * 2;
            REQUIRE(buffer.x() == 4);
            REQUIRE(buffer.y() == 6);
            REQUIRE(buffer.z() == 8);
        }

        {
            XYZBuffer<int> buffer{2, 3, 4};
            buffer *= 2;
            REQUIRE(buffer.x() == 4);
            REQUIRE(buffer.y() == 6);
            REQUIRE(buffer.z() == 8);

            buffer *= buffer;
            REQUIRE(buffer.x() == 16);
            REQUIRE(buffer.y() == 36);
            REQUIRE(buffer.z() == 64);
        }
    }

    SECTION("Test Operator: /")
    {
        std::cout << "    Section 'Test Operator: /'" << std::endl;   
        {
            auto buffer = XYZBuffer<int>{1, 1, 2} / XYZBuffer<int>{1, 1, 2};
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 1);
            REQUIRE(buffer.z() == 1);
        }

        {
            auto buffer = XYZBuffer<int>{2, 3, 4} / 1;
            REQUIRE(buffer.x() == 2);
            REQUIRE(buffer.y() == 3);
            REQUIRE(buffer.z() == 4);
        }

        {
            XYZBuffer<int> buffer{2, 4, 6};
            buffer /= 2;
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 2);
            REQUIRE(buffer.z() == 3);

            buffer /= buffer;
            REQUIRE(buffer.x() == 1);
            REQUIRE(buffer.y() == 1);
            REQUIRE(buffer.z() == 1);
        }
    }
}
