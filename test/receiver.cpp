#include <string>
#include <chrono>
#include <thread>
#include <iostream>

#include <zmq.hpp>
#include <msg/pose.h>
#include <comm/receiver.h>

using fastsense::msg::Pose;

int main() 
{
    Receiver<Pose> receiver(5555);

    for (;;) 
    {
        auto p = receiver.receive();
        std::cout << p.x << "/" << p.y << "/" << p.z << "\n";
    }

    return 0;
}