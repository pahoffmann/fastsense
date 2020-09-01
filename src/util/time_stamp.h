//
// Created by julian on 9/1/20.
//

#pragma once

#include <chrono>

namespace fastsense::util
{

struct TimeStamp
{
    TimeStamp();
    void updateTime();
    std::chrono::high_resolution_clock::time_point time;
};

} // namespace fastsense::util