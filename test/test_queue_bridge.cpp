/**
 * @file communication.cpp
 * @author Marcel Flottmann
 * @date 2020-10-6
 */

#include "catch2_config.h"
#include <comm/receiver.h>
#include <comm/queue_bridge.h>
#include <iostream>
#include <thread>

using namespace fastsense::comm;
using namespace fastsense::msg;
using namespace fastsense::util;

TEST_CASE("QueueBridge", "[communication]")
{
    std::cout << "Testing 'QueueBridge'" << std::endl;
    int value_received = 0;
    int value_to_send = 42;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<int>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<int>>(16);
    QueueBridge<int, true> bridge{in, out, 1234};

    std::thread receive_thread{[&]()
    {
        Receiver<int> receiver{"127.0.0.1", 1234};
        std::this_thread::sleep_for(std::chrono::seconds(2));
        value_received = receiver.receive();
        received = true;
    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(value_to_send);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }};

    bridge.start();
    while(!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();
    REQUIRE(value_to_send == value_received);
    REQUIRE(out->getLength() != 0);
}

TEST_CASE("QueueBridge shared_ptr", "[communication]")
{
    std::cout << "Testing 'QueueBridge shared_ptr'" << std::endl;
    int value_received = 0;
    int value_to_send = 42;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<std::shared_ptr<int>>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<std::shared_ptr<int>>>(16);
    QueueBridge<std::shared_ptr<int>, true> bridge{in, out, 1234};

    std::thread receive_thread{[&]()
    {
        Receiver<int> receiver{"127.0.0.1", 1234};
        std::this_thread::sleep_for(std::chrono::seconds(2));
        value_received = receiver.receive();
        received = true;
    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(std::make_shared<int>(value_to_send));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }};

    bridge.start();
    while(!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();
    REQUIRE(value_to_send == value_received);
    REQUIRE(out->getLength() != 0);
}