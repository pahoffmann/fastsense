#include "catch2_config.h"
#include <msg/imu.h>
#include <driver/imu/filter.h>

using namespace fastsense;
using namespace fastsense::driver;

TEST_CASE("Moving Average Filter", "[MovAvgFilter]")
{
    {
        size_t window_size = 0;
        double mean = 0;

        SlidingWindowFilter<double> filter(window_size);

        mean = filter.update(1.);
        REQUIRE(mean == 1.);

        mean = filter.update(2.);
        REQUIRE(mean == 2.);

        mean = filter.update(3.);
        REQUIRE(mean == 3.);
    }
    {
        double window_size = 3;
        double mean = 0;

        SlidingWindowFilter<double> filter(window_size);
        mean = filter.update(1.);
        REQUIRE(mean == 1.);

        mean = filter.update(2.);
        REQUIRE(mean == 2.);

        mean = filter.update(2.);
        REQUIRE(mean == 2.);

        // Buffer is filled for first time
        REQUIRE(filter.get_buffer().size() == window_size);

        // Mean from first three steps is (internally)
        // (((0 + 1/3) + 2/3) + 2/3) ~ 1.66667
        REQUIRE(filter.get_mean() == (((0. + 1./3.) + 2./3.) + 2./3.));

        // Now actual sliding window averaging begins: more than window_size measurements

        // front(): 1.
        REQUIRE(filter.get_buffer().front() == 1.);
        // Update with 3: mean += back (now 3) - front (1) / window_size => 2.33333
        auto old_mean = filter.get_mean();
        mean = filter.update(3.);
        REQUIRE(mean == old_mean + (3. - 1.)/window_size);
        // back(): 3, as just declared
        REQUIRE(filter.get_buffer().back() == 3.);

        // front(): 2.
        REQUIRE(filter.get_buffer().front() == 2.);
        // Update with 2: mean (2.33333) += (back (now 2) - front(now 2)/3)
        old_mean = filter.get_mean();
        mean = filter.update(2.);
        REQUIRE(mean == old_mean);
        // back(): 2.
        REQUIRE(filter.get_buffer().back() == 2.);
    }

    {
//        MovingAverageFilter<msg::Imu> filter;
//        filter.update(msg::Imu{msg::LinearAcceleration(1, 1, 1), msg::AngularVelocity(1, 1, 1), msg::MagneticField(1, 1, 1)});
//        filter.update(msg::Imu{msg::LinearAcceleration(2, 2, 2), msg::AngularVelocity(2, 2, 2), msg::MagneticField(2, 2, 2)});
//        auto mean = filter.update(msg::Imu{msg::LinearAcceleration(3, 3, 3), msg::AngularVelocity(3, 3, 3), msg::MagneticField(3, 3, 3)});
//
//        REQUIRE(mean.acc[0] == 2);
//        REQUIRE(mean.acc[1] == 2);
//        REQUIRE(mean.acc[2] == 2);
//        REQUIRE(mean.ang[0] == 2);
//        REQUIRE(mean.ang[1] == 2);
//        REQUIRE(mean.ang[2] == 2);
//        REQUIRE(mean.mag[0] == 2);
//        REQUIRE(mean.mag[1] == 2);
//        REQUIRE(mean.mag[2] == 2);
    }
}