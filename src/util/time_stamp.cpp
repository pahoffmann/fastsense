//
// Created by julian on 9/1/20.
//

#include <util/time_stamp.h>

using namespace fastsense::util;

TimeStamp::TimeStamp() : time(std::chrono::high_resolution_clock::now()) {}

int TimeStamp::operator-(const TimeStamp& t)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(time - t.time).count();
}

void TimeStamp::updateTime()
{
    time = std::chrono::high_resolution_clock::now();
}


