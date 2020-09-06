#include <string>
#include <iostream>

#include <zmq.hpp>
#include <msg/point_cloud.h>

using fastsense::util::msg::Point;

int main()
{
    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // construct a REQ (request) socket and connect to interface
    zmq::socket_t socket{context, zmq::socket_type::push};
    socket.connect("tcp://localhost:5555");

    // set up some static data to send
    const std::string data{"Hello"};

    Point p;
    p.x = 1;
    p.y = 2;
    p.z = 3;

    for (auto request_num = 0; request_num < 10; ++request_num) 
    {
        // send the request message
        std::cout << "Sending Point " << request_num << "..." << std::endl;
        socket.send(&p, sizeof(p));
    }

    return 0;
}