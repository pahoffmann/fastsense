//
// Created by julian on 9/1/20.
//

#include <util/time_stamp.h>

using namespace fastsense::util;

TimeStamp::TimeStamp() : time(std::chrono::high_resolution_clock::now()) {}

void TimeStamp::updateTime()
{
    time = std::chrono::high_resolution_clock::now();
}


