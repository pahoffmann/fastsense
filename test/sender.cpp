#include <string>
#include <iostream>

#include <zmq.hpp>
#include <msg/pose.h>
#include <comm/sender.h>

#include "catch2_config.h"

using fastsense::msg::Point;
using fastsense::comm::Sender;

TEST_CASE("Send 10 Points", "[!hide]")
{
    Sender<Point> sender("localhost:5555");

    Point p;
    p.x = 1;
    p.y = 2;
    p.z = 3;

    for (auto request_num = 0; request_num < 10; ++request_num)
    {
        // send the request message
        std::cout << "Sending Point " << request_num << "..." << std::endl;
        sender.send(&p);
    }
}