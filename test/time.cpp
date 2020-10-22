/**
 * @author Steffen Hinderink
 */

#include "catch2_config.h"
#include "util/runtime_evaluator.h"

#include <thread> // for sleeping
#include <iostream> // for output

using namespace fastsense::util;

TEST_CASE("Time", "[Time]")
{
    std::cout << "Testing 'Time'" << std::endl;

    #ifdef TIME_MEASUREMENT
    auto& re = RuntimeEvaluator::get_instance();

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

    // TODO: check results
    CHECK(true);
    #endif

    #ifndef TIME_MEASUREMENT
    std::cout << "Time measurements are disabled (TIME_MEASUREMENT in runtime_evaluator.h is not defined)" << std::endl;
    CHECK(true);
    #endif
}
