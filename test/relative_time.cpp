/**
 * @author Julian Gaal
 */

#include "catch2_config.h"
#include "util/time.h"
#include <thread>
#include <iostream>

using namespace fastsense::util;
using namespace std::chrono_literals;

TEST_CASE("Test relative time", "[relative_time]")
{
	RelativeTime::init();
	std::this_thread::sleep_for(0.5s);
	REQUIRE(RelativeTime::now() == Approx(std::chrono::duration_cast<std::chrono::milliseconds>(0.5s).count()).margin(10));
	std::this_thread::sleep_for(0.5s);
	REQUIRE(RelativeTime::now() == Approx(std::chrono::duration_cast<std::chrono::milliseconds>(1s).count()).margin(10));
	std::this_thread::sleep_for(0.5s);
	REQUIRE(RelativeTime::now() == Approx(std::chrono::duration_cast<std::chrono::milliseconds>(1.5s).count()).margin(10));


	std::thread t1([]()
    {
	    std::this_thread::sleep_for(1s);
	    REQUIRE(RelativeTime::now() == Approx(std::chrono::duration_cast<std::chrono::milliseconds>(2.5s).count()).margin(15));
    });

	std::thread t2([]()
    {
	    std::this_thread::sleep_for(2s);
	    REQUIRE(RelativeTime::now() == Approx(std::chrono::duration_cast<std::chrono::milliseconds>(3.5s).count()).margin(15));
    });
	
	t1.join();
	t2.join();
}