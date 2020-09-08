#define CATCH_CONFIG_MAIN

#include <string>
#include <chrono>
#include <thread>
#include <iostream>

#include <zmq.hpp>
#include <msg/pose.h>
#include <comm/receiver.h>

#include <catch2/catch.hpp>

using fastsense::msg::Point;

TEST_CASE("Receiver", "")
{
    Receiver<Point> receiver(5555);

    for (;;) 
    {
        auto p = receiver.receive();
        REQUIRE(p.x == 1);
        REQUIRE(p.y == 2);
        REQUIRE(p.z == 3);
    }
}