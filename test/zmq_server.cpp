#include <string>
#include <chrono>
#include <thread>
#include <iostream>

#include <zmq.hpp>
#include <util/msg/point_cloud.h>

using fastsense::util::msg::Point;

int main() 
{
    using namespace std::chrono_literals;

    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // construct a REP (reply) socket and bind to interface
    zmq::socket_t socket{context, zmq::socket_type::pull};
    socket.bind("tcp://*:5555");

    // prepare some static data for responses
    const std::string data{"World"};

    for (;;) 
    {
        zmq::message_t msg; 

        // receive a request from client
        socket.recv(msg, zmq::recv_flags::none);
        Point* p = (Point*)msg.data();
        std::cout << "Received " << p->x << "/" << p->y << "/" << p->z << std::endl;
    }

    return 0;
}