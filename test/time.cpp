/**
 * @author Steffen Hinderink
 */

#include "catch2_config.h"
#include "util/runtime_evaluator.h"

#include <thread> // for sleeping
#include <iostream> // for output

using namespace fastsense::util;

/**
 * Tests a time value with Catch2. In order for the test to succeed the time must be in an interval.
 * The lower bound is given, the time that has certainly passed can be used for that.
 * The upper bound is calculated from the lower bound. A deviation of 10% is deemed acceptable.
 * Additionally a small constant time is added to the upper bound to make up for measurement inaccuracies of short times.
 * @param time Time value to be checked
 * @param lower_bound Lower bound of the interval in which the time has to be
 */
void check_time(unsigned long long time, unsigned long long lower_bound)
{
    CHECK(time >= lower_bound);
    CHECK(time < 1.1 * lower_bound + 1000);
}

TEST_CASE("Time", "[Time]")
{
    std::cout << "Testing 'Time'" << std::endl;

#ifdef TIME_MEASUREMENT
    auto& re = RuntimeEvaluator::get_instance();
    re.clear();

    re.start("outer total");
    for (int i = 1; i <= 10; i++)
    {
        re.start("inner total");

        re.start("test 1");
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        re.stop("test 1");

        re.start("test 2");
        std::this_thread::sleep_for(std::chrono::milliseconds(i));
        re.stop("test 2");

        re.stop("inner total");
    }
    re.stop("outer total");

    re.start("unstopped");

    std::cout << re << std::endl;

    // Check results
    auto forms = re.get_forms();
    for (const auto& ef : forms)
    {
        if (ef.name == "outer total")
        {
            CHECK(!ef.active);
            CHECK(ef.count == 1);
            // 10 * 21 + \sum_{i = 1}^{10} i = 10 * 21 + 10 * 11 / 2 = 210 + 55 = 265
            check_time(ef.last, 256000);
            check_time(ef.sum, 256000);
            check_time(ef.min, 256000);
            check_time(ef.max, 256000);
            CHECK(ef.min <= ef.max);
        }
        else if (ef.name == "inner total")
        {
            CHECK(!ef.active);
            CHECK(ef.count == 10);
            // 21 + 10 = 31
            check_time(ef.last, 31000);
            // 10 * 21 + \sum_{i = 1}^{10} i = 10 * 21 + 10 * 11 / 2 = 210 + 55 = 265
            check_time(ef.sum, 256000);
            // 21 + 1 = 22
            check_time(ef.min, 22000);
            // 21 + 10 = 31
            check_time(ef.max, 31000);
            CHECK(ef.min <= ef.max);
        }
        else if (ef.name == "test 1")
        {
            CHECK(!ef.active);
            CHECK(ef.count == 10);
            // 21
            check_time(ef.last, 21000);
            // 10 * 21 = 210
            check_time(ef.sum, 210000);
            // 21
            check_time(ef.min, 21000);
            // 21
            check_time(ef.max, 21000);
            CHECK(ef.min <= ef.max);
        }
        else if (ef.name == "test 2")
        {
            CHECK(!ef.active);
            CHECK(ef.count == 10);
            // 10
            check_time(ef.last, 10000);
            // \sum_{i = 1}^{10} i = 10 * 11 / 2 = 55
            check_time(ef.sum, 55000);
            // 1
            check_time(ef.min, 1000);
            // 10
            check_time(ef.max, 10000);
            CHECK(ef.min <= ef.max);
        }
        else if (ef.name == "unstopped")
        {
            CHECK(ef.active);
            CHECK(ef.count == 0);
            CHECK(ef.last == 0);
            CHECK(ef.sum == 0);
            CHECK(ef.min == std::numeric_limits<unsigned long long>::max());
            CHECK(ef.max == 0);
        }
        else
        {
            CHECK(false); // no other measurements were made
        }
    }
#endif

#ifndef TIME_MEASUREMENT
    std::cout << "Time measurements are disabled (TIME_MEASUREMENT in runtime_evaluator.h is not defined)" << std::endl;
    CHECK(true);
#endif
}
