/**
 * @author Steffen Hinderink
 */

#include "catch2_config.h"
#include "util/runtime_evaluator.h"
#include <chrono>
#include <thread>

using namespace fastsense::util;

TEST_CASE("Time", "[Time]")
{
    std::cout << "Testing 'Time'" << std::endl;

    #ifdef TIME_MEASUREMENT
    #endif

    int sum = 0;
    
    for (int i = 1; i <= 3; i++)
    {
        sum += i;

        auto& re = RuntimeEvaluator::get_instance();
        re.start("test");

        std::this_thread::sleep_for(std::chrono::milliseconds(21));
    
        re.stop();
    }

    std::cout << RuntimeEvaluator::get_instance() << std::endl;

    std::cout << sum << std::endl;

    CHECK(true);
}
