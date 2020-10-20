/**
 * @author Steffen Hinderink
 */

#include "catch2_config.h"
#include "util/runtime_evaluator.h"

using namespace fastsense::util;

TEST_CASE("Time", "[Time]")
{
    std::cout << "Testing 'Time'" << std::endl;
    #ifdef TIME_MEASUREMENT
    auto re = RuntimeEvaluator::get_instance();
    std::cout << "start" << std::endl;
    re.start("test");
    #endif

    #ifdef TIME_MEASUREMENT
    re.stop();
    std::cout << "stop" << std::endl;
    std::cout << re << std::endl;
    #endif

    CHECK(true);
}
