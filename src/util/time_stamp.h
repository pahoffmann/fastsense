//
// Created by julian on 9/1/20.
//

#pragma once

#include <chrono>
#include <tuple>

namespace fastsense::util
{

struct TimeStamp
{
    TimeStamp();
    void reset();

    int operator-(const TimeStamp& t);

    std::chrono::steady_clock::time_point time;
};

} // namespace fastsense::util