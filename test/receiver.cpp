#include <string>
#include <chrono>
#include <thread>
#include <iostream>

#include <zmq.hpp>
#include <msg/pose.h>
#include <comm/receiver.h>

#include "catch2_config.h"

using fastsense::msg::Point;
using namespace fastsense::comm;

TEST_CASE("Receive 10 Points", "[!hide]")
{
    std::cout << "Testing 'Receive 10 Points'" << std::endl;
    Receiver<Point> receiver(5555);

    for (int i = 0; i < 10; i++)
    {
        auto p = receiver.receive();
        REQUIRE(p.x == 1);
        REQUIRE(p.y == 2);
        REQUIRE(p.z == 3);
    }
}