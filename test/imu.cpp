#include "catch2_config.h"
#include <msg/imu.h>
#include <driver/imu/filter.h>

using namespace fastsense;
using namespace fastsense::driver;

TEST_CASE("Moving Average Filter", "[MovAvgFilter]")
{
    {
        MovingAverageFilter<double> filter;
        filter.update(1.f);
        filter.update(2.f);
        auto mean = filter.update(3.f);

        REQUIRE(mean == 2.f);
    }

    {
        MovingAverageFilter<msg::Imu> filter;
        filter.update(msg::Imu{msg::LinearAcceleration(1, 1, 1), msg::AngularVelocity(1, 1, 1), msg::MagneticField(1, 1, 1)});
        filter.update(msg::Imu{msg::LinearAcceleration(2, 2, 2), msg::AngularVelocity(2, 2, 2), msg::MagneticField(2, 2, 2)});
        auto mean = filter.update(msg::Imu{msg::LinearAcceleration(3, 3, 3), msg::AngularVelocity(3, 3, 3), msg::MagneticField(3, 3, 3)});

        REQUIRE(mean.acc[0] == 2);
        REQUIRE(mean.acc[1] == 2);
        REQUIRE(mean.acc[2] == 2);
        REQUIRE(mean.ang[0] == 2);
        REQUIRE(mean.ang[1] == 2);
        REQUIRE(mean.ang[2] == 2);
        REQUIRE(mean.mag[0] == 2);
        REQUIRE(mean.mag[1] == 2);
        REQUIRE(mean.mag[2] == 2);
    }
}