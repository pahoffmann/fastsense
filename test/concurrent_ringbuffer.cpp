/**
 * @file concurrent_ringbuffer.cpp
 * @author Juri Vana
 * @date 2020-10-13
 */

#include "catch2_config.h"
#include <util/concurrent_ring_buffer.h>
#include <iostream>
#include <thread>

using namespace fastsense::util;

TEST_CASE("ConcRingBuffer", "[ConcRingBuffer]")
{
    std::cout << "Testing 'Concurrent Ring Buffer'" << std::endl;
    constexpr size_t buffer_size = 5;

    ConcurrentRingBuffer<int> crb(buffer_size);

    // test getLength and push
    REQUIRE(crb.getLength() == 0);

    for (uint i = 0; i < buffer_size; i++)
    {
        crb.push(i);
    }

    REQUIRE(crb.getLength() == buffer_size);

    SECTION("Test push_nb, pop, and pop_nb")
    {
        // test push_nb, pop
        REQUIRE(!crb.push_nb(0));
        REQUIRE(crb.push_nb(buffer_size, true));

        int val;
        crb.pop(&val);
        REQUIRE(val == 1);
        REQUIRE(crb.getLength() == buffer_size - 1);

        // test pop_nb
        for (int i = 2; i <= (int) buffer_size; i++)
        {
            REQUIRE(crb.pop_nb(&val));
            REQUIRE(val == i);
            REQUIRE(crb.getLength() == buffer_size - i);
        }

        REQUIRE(crb.getLength() == 0);
        REQUIRE(!crb.pop_nb(&val));
    }

    SECTION("Test clear")
    {
        crb.clear();
        REQUIRE(crb.getLength() == 0);
    }

    SECTION("Test multithreading")
    {
        int val;
        size_t length;

        // test waiting to push
        std::thread push_thread{[&]()
        {
            // wait for other thread to pop
            crb.push(buffer_size);
            length = crb.getLength();
        }};

        std::thread pop_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            crb.pop(&val);
        }};

        push_thread.join();
        pop_thread.join();

        REQUIRE(length == buffer_size);
        REQUIRE(val == 0);

        // test waiting to pop
        crb.clear();

        std::thread push_thread2{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            crb.push(1);
        }};

        std::thread pop_thread2{[&]()
        {
            // wait for other thread to push
            crb.pop(&val);
            length = crb.getLength();
        }};

        push_thread2.join();
        pop_thread2.join();

        REQUIRE(length == 0);
        REQUIRE(val == 1);
    }
}
