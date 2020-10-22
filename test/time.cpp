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

    for (int i = 1; i <= 100; i++)
    {
        #ifdef TIME_MEASUREMENT
        auto& re = RuntimeEvaluator::get_instance();
        re.start("test1");
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(21));

        #ifdef TIME_MEASUREMENT
        re.stop("test1");
        #endif

        #ifdef TIME_MEASUREMENT
        re.start("test2");
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(i));

        #ifdef TIME_MEASUREMENT
        re.stop("test2");
        #endif
    }

    #ifdef TIME_MEASUREMENT
    std::cout << RuntimeEvaluator::get_instance() << std::endl;
    #endif

    CHECK(true);
}
